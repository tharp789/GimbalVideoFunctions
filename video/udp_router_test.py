import gi
import sys
import threading

import router_helpers as rh

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# === Configuration Parameters ===
VIDEO_SOURCE_URL = "rtsp://192.168.144.25:8554/video1"
VIDEO_SINK_URL = "rtsp://orin-nano-10:8554/test/rgb"
SOURCE_BUFFER_MS = 0
SINK_BUFFER_MS = 0
DROP_ON_LATENCY = True
CODEC = 1  # 1 = H264, 2 = H265
TARGET_WIDTH = 1920
TARGET_HEIGHT = 1080
TARGET_BITRATE_KBPS = 4000

def on_bus_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End-Of-Stream reached.")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, dbg = message.parse_error()
        print(f"Error: {err.message}")
        loop.quit()

def main():
    Gst.init(None)
    pipeline = Gst.Pipeline.new("video_pipeline")

    def make(name, type_name=None):
        element = Gst.ElementFactory.make(name, type_name or name)
        if not element:
            print(f"Failed to create element: {name}")
            sys.exit(1)
        return element

    # Elements
    rtspsrc = make("rtspsrc", "videosrc")
    rtspsrc.set_property("location", VIDEO_SOURCE_URL)
    rtspsrc.set_property("is-live", True)
    rtspsrc.set_property("latency", SOURCE_BUFFER_MS)
    rtspsrc.set_property("drop-on-latency", DROP_ON_LATENCY)
    rtspsrc.set_property("protocols", "tcp")

    queue1 = make("queue", "queue1")
    queue2 = make("queue", "queue2")
    queue3 = make("queue", "queue3")
    queue4 = make("queue", "queue4")
    queue5 = make("queue", "queue5")
    queue6 = make("queue", "queue6")
    queue7 = make("queue", "queue7")
    queue8 = make("queue", "queue8")

    for q in [queue1, queue2, queue3, queue4, queue5, queue6, queue7, queue8]:
        q.set_property("leaky", 2)

    depay = make("rtph265depay" if CODEC == 2 else "rtph264depay", "depay")
    parse = make("h265parse" if CODEC == 2 else "h264parse", "parse")

    decoder = make("avdec_h264" if CODEC == 1 else "avdec_h265", "decoder")
    convert = make("videoconvert", "convert")
    scale = make("videoscale", "scale")

    capsfilter = make("capsfilter", "capsfilter")
    caps = Gst.Caps.from_string(f"video/x-raw,width={TARGET_WIDTH},height={TARGET_HEIGHT}")
    capsfilter.set_property("caps", caps)

    encoder = make("x264enc", "encoder")
    encoder.set_property("bitrate", TARGET_BITRATE_KBPS)
    encoder.set_property("speed-preset", "ultrafast")
    encoder.set_property("tune", "zerolatency")
    encoder.set_property("key-int-max", 30)

    out_parse = make("h264parse" if CODEC == 1 else "h265parse", "out_parse")
    identity = make("identity", "identity")
    identity.set_property("sync", True)
    identity.get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, rh.bitrate_probe_callback, None)

    # videosink = make("rtspclientsink", "videosink")
    # videosink.set_property("location", VIDEO_SINK_URL)
    # videosink.set_property("latency", SINK_BUFFER_MS)
    rtphpayload = make("rtph264pay" if CODEC == 1 else "rtph265pay", "rtph264pay")
    rtphpayload.set_property("config-interval", 1)

    videosink = make("udpsink", "videosink")
    videosink.set_property("host", "orin-nano-10")
    videosink.set_property("port", 8000)

    # Add to pipeline
    for elem in [
        rtspsrc, queue1, depay, queue2, parse, queue3, decoder, queue4,
        convert, scale, capsfilter, queue5, encoder, queue6, out_parse,
        queue7, identity, queue8, rtphpayload, videosink]:
        pipeline.add(elem)

    # Link elements
    rtspsrc.connect("pad-added", rh.on_pad_added, queue1)
    queue1.link(depay)
    depay.link(queue2)
    queue2.link(parse)
    parse.link(queue3)
    queue3.link(decoder)
    decoder.link(queue4)
    queue4.link(convert)
    convert.link(scale)
    scale.link(capsfilter)
    capsfilter.link(queue5)
    queue5.link(encoder)
    encoder.link(queue6)
    queue6.link(out_parse)
    out_parse.link(queue7)
    queue7.link(identity)
    identity.link(queue8)
    queue8.link(rtphpayload)
    rtphpayload.link(videosink)

    # Bus
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message)

    # Run pipeline
    print("Starting pipeline...")
    pipeline.set_state(Gst.State.PLAYING)

    # Start CPU logger thread
    stop_event = threading.Event()
    samples = []
    codec_type, width, height, bitrate = rh.get_codec()
    log_file_name = f"compute_logs/simple_{codec_type.lower()}_{width}x{height}_{bitrate}_cpu_usage.log"
    logger_thread = threading.Thread(target=rh.cpu_logger, args=(stop_event, log_file_name, samples))
    logger_thread.start()

    try:
        global loop
        loop = GLib.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        print("Interrupted: stopping pipeline...")
    finally:
        stop_event.set()
        logger_thread.join()
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")
        
if __name__ == "__main__":
    main()

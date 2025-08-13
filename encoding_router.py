import gi
import sys
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

# === Configuration ===
VIDEO_SOURCE_URL = "rtsp://your-input-stream"
VIDEO_SINK_URL = "rtsp://your-output-destination"
SOURCE_BUFFER_MS = 200
SINK_BUFFER_MS = 200
DROP_ON_LATENCY = True
CODEC = 1  # 1 = H264, 2 = H265
TARGET_WIDTH = 640
TARGET_HEIGHT = 480
TARGET_BITRATE_KBPS = 500

# === Callbacks ===
def on_pad_added(src, new_pad, target_element):
    sink_pad = target_element.get_static_pad("sink")
    if not sink_pad.is_linked():
        result = new_pad.link(sink_pad)
        if result != Gst.PadLinkReturn.OK:
            print("Failed to link RTSP pad")
        else:
            print("Linked RTSP pad successfully")

def bitrate_probe_callback(pad, info, user_data):
    # Stub: You can monitor bitrate here
    return Gst.PadProbeReturn.OK

def on_bus_message(bus, message):
    t = message.type
    if t == Gst.MessageType.EOS:
        print("End-Of-Stream reached.")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, dbg = message.parse_error()
        print(f"Error: {err.message}")
        loop.quit()

# === Main Setup ===
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
    rtspsrc.set_property("udp-buffer-size", 212992)
    rtspsrc.set_property("timeout", 0)
    rtspsrc.set_property("tcp-timeout", 0)

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

    out_parse = make("h264parse" if CODEC == 1 else "h265parse", "out_parse")
    identity = make("identity", "identity")
    identity.set_property("sync", True)
    identity.get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, bitrate_probe_callback, None)

    videosink = make("rtspclientsink", "videosink")
    videosink.set_property("location", VIDEO_SINK_URL)
    videosink.set_property("latency", SINK_BUFFER_MS)
    videosink.set_property("timeout", 0)
    videosink.set_property("tcp-timeout", 0)

    # Add to pipeline
    elements = [
        rtspsrc, queue1, depay, queue2, parse, queue3, decoder, queue4,
        convert, scale, capsfilter, queue5, encoder, queue6, out_parse,
        queue7, identity, queue8, videosink
    ]

    for e in elements:
        pipeline.add(e)

    # Link pipeline (manual for rtspsrc)
    rtspsrc.connect("pad-added", on_pad_added, queue1)
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
    queue8.link(videosink)

    # Bus
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message)

    # Run pipeline
    print("Starting pipeline...")
    pipeline.set_state(Gst.State.PLAYING)

    try:
        global loop
        loop = GObject.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt: stopping pipeline...")
    finally:
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")

if __name__ == "__main__":
    main()

import gi
import sys
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# === Configuration Parameters ===
VIDEO_SOURCE_URL = "rtsp://192.168.144.25:8554/video1"
VIDEO_SINK_URL = "rtsp://orin-nano-10:8554/test/rgb"
SOURCE_BUFFER_MS = 0
SINK_BUFFER_MS = 0
DROP_ON_LATENCY = True
CODEC = 1  # 1 = H264, 2 = H265

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

# === Main Pipeline Setup ===
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

    for q in [queue1, queue2, queue3, queue4]:
        q.set_property("leaky", 2)

    depay = make("rtph265depay" if CODEC == 2 else "rtph264depay", "depay")
    parse = make("h265parse" if CODEC == 2 else "h264parse", "parse")

    identity = make("identity", "identity")
    identity.set_property("sync", True)
    identity.get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, bitrate_probe_callback, None)

    videosink = make("rtspclientsink", "videosink")
    videosink.set_property("location", VIDEO_SINK_URL)
    videosink.set_property("latency", SINK_BUFFER_MS)
    videosink.set_property("timeout", 0)
    videosink.set_property("tcp-timeout", 0)

    # Add elements to pipeline
    for elem in [rtspsrc, queue1, depay, queue2, parse, queue3, identity, queue4, videosink]:
        pipeline.add(elem)

    # Link elements
    rtspsrc.connect("pad-added", on_pad_added, queue1)
    queue1.link(depay)
    depay.link(queue2)
    queue2.link(parse)
    parse.link(queue3)
    queue3.link(identity)
    identity.link(queue4)
    queue4.link(videosink)

    # Bus
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", on_bus_message)

    # Run pipeline
    print("Starting pipeline...")
    pipeline.set_state(Gst.State.PLAYING)

    try:
        global loop
        loop = GLib.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        print("Interrupted: stopping pipeline...")
    finally:
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")

if __name__ == "__main__":
    main()

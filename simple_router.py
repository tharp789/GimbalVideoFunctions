import gi
import sys
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

# === Configuration Parameters ===
VIDEO_SOURCE_URL = "rtsp://192.168.144.25:8554/video1"
VIDEO_SINK_URL = "rtsp://tyler-MS-06:8554/test"
SOURCE_BUFFER_MS = 200  # example value
SINK_BUFFER_MS = 200  # example value
DROP_ON_LATENCY = True
CODEC = 1  # 1 for H264, 2 for H265

# === Callback Stubs ===
def on_pad_added(src, new_pad, target_element):
    sink_pad = target_element.get_static_pad("sink")
    if not sink_pad.is_linked():
        new_pad.link(sink_pad)

def bitrate_probe_callback(pad, info, user_data):
    # Add logic here if needed
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

    # Create and configure elements
    def make_element(factory_name, name=None):
        elem = Gst.ElementFactory.make(factory_name, name)
        if not elem:
            print(f"Failed to create element: {factory_name}")
            sys.exit(1)
        return elem

    rtspsrc = make_element("rtspsrc", "videosrc")
    rtspsrc.set_property("location", VIDEO_SOURCE_URL)
    rtspsrc.set_property("is-live", True)
    rtspsrc.set_property("latency", SOURCE_BUFFER_MS)
    rtspsrc.set_property("drop-on-latency", DROP_ON_LATENCY)
    rtspsrc.set_property("udp-buffer-size", 212992)
    rtspsrc.set_property("timeout", 0)
    rtspsrc.set_property("tcp-timeout", 0)

    queue1 = make_element("queue", "queue1")
    queue1.set_property("leaky", 2)

    if CODEC == 2:
        rtphdepay = make_element("rtph265depay", "rtphdepay")
        rtphdepay_caps = Gst.caps_from_string("video/x-h265, stream-format=byte-stream, alignment=au")
        hparse = make_element("h265parse", "hparse")
    else:
        rtphdepay = make_element("rtph264depay", "rtphdepay")
        rtphdepay_caps = Gst.caps_from_string("video/x-h264, stream-format=byte-stream, alignment=au")
        hparse = make_element("h264parse", "hparse")
    hparse_caps = rtphdepay_caps  # same caps for parse step

    queue2 = make_element("queue", "queue2")
    queue2.set_property("leaky", 2)

    queue3 = make_element("queue", "queue3")
    queue3.set_property("leaky", 2)

    identity = make_element("identity", "identity")
    identity.set_property("sync", True)
    identity.get_static_pad("sink").add_probe(
        Gst.PadProbeType.BUFFER, bitrate_probe_callback, None
    )

    queue4 = make_element("queue", "queue4")
    queue4.set_property("leaky", 2)

    videosink = make_element("rtspclientsink", "videosink")
    videosink.set_property("location", VIDEO_SINK_URL)
    videosink.set_property("timeout", 0)
    videosink.set_property("tcp-timeout", 0)
    videosink.set_property("latency", SINK_BUFFER_MS)

    # Add elements to pipeline
    for elem in [rtspsrc, queue1, rtphdepay, queue2, hparse, queue3, identity, queue4, videosink]:
        pipeline.add(elem)

    # Link elements
    rtspsrc.connect("pad-added", on_pad_added, queue1)
    queue1.link(rtphdepay)
    rtphdepay.link_filtered(queue2, rtphdepay_caps)
    queue2.link(hparse)
    hparse.link_filtered(queue3, hparse_caps)
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

    # Run main loop
    try:
        global loop
        loop = GObject.MainLoop()
        loop.run()
    except KeyboardInterrupt:
        print("Interrupted: stopping pipeline...")
    finally:
        pipeline.set_state(Gst.State.NULL)
        print("Pipeline stopped.")

if __name__ == "__main__":
    main()

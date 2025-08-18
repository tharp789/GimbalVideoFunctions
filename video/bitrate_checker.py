import gi
import struct
import time
import os
import argparse

## EXAMPLE USAGE
# python3 bandwidth_check.py --device_id 88 --modality rgb

gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GLib

os.environ["GST_DEBUG"] = "3"

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device_id", type=int, default=0)
    parser.add_argument("--modality", type=str, default="rgb")
    return parser.parse_args()

def on_buffer(pad, info):
    global total_bytes
    buffer = info.get_buffer()
    if buffer:
        total_bytes += buffer.get_size()
    return Gst.PadProbeReturn.OK

bandwidth_buffer = []
def print_bandwidth():
    global total_bytes, last_time
    now = time.time()
    duration = now - last_time
    if duration > 0:
        bandwidth_bps = (total_bytes * 8) / duration
        bandwidth_mbps = bandwidth_bps / 1_000_000
        bandwidth_buffer.append(bandwidth_mbps)
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
        print(f"[{timestamp}] Bandwidth: {bandwidth_mbps:.2f} Mbps")
        with open("bandwidth_log.txt", "a") as f:
            f.write(f"{timestamp}, {bandwidth_mbps:.2f} Mbps\n")
    total_bytes = 0
    last_time = now
    return True  # Keep calling

def on_pad_added(rtspsrc, new_pad, element):
    sink_pad = element.get_static_pad("sink")
    if sink_pad.is_linked():
        print("Pad already linked. Exiting.")
        return
    res = new_pad.link(sink_pad)
    if res == Gst.PadLinkReturn.OK:
        print("Pad linked successfully.")
    else:
        print("Pad linking failed.")



def main(stream_url):
    Gst.init(None)

    # Create pipeline
    pipeline = Gst.Pipeline.new("timestamp_decoding_pipeline")

    # Create elements
    rtspsrc = Gst.ElementFactory.make("rtspsrc", "rtspsrc")
    rtspsrc.set_property("location", stream_url)
    rtspsrc.set_property("is-live", True) 
    rtspsrc.set_property("latency", 0)
    rtspsrc.set_property("drop-on-latency", True)

    identity = Gst.ElementFactory.make("identity", "identity")
    identity.set_property("sync", False)
    identity.set_property("silent", True)
    bandwidth_pad = identity.get_static_pad("src")
    bandwidth_pad.add_probe(Gst.PadProbeType.BUFFER, on_buffer)

    fakesink = Gst.ElementFactory.make("fakesink", "fakesink")
    fakesink.set_property("sync", False)
    fakesink.set_property("silent", True)

    # Add all elements to pipeline
    for element in [rtspsrc, identity, fakesink]:
        pipeline.add(element)

    # Link elements
    rtspsrc.connect("pad-added", on_pad_added, identity)
    identity.link(fakesink)

    GLib.timeout_add_seconds(1, print_bandwidth)

    # Set pipeline to playing and run main loop
    pipeline.set_state(Gst.State.PLAYING)
    mainloop = GLib.MainLoop()

    try:
        mainloop.run()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.set_state(Gst.State.NULL)
        # print summary stats of bandwidth buffer 
        print("--------------------------------")
        print("[Non Zero] Bandwidth Buffer Stats")
        non_zero_buffer = [x for x in bandwidth_buffer if x > 0]
        print(f"Count: {len(non_zero_buffer)}, Min: {min(non_zero_buffer)}, Max: {max(non_zero_buffer)}, Avg: {sum(non_zero_buffer) / len(non_zero_buffer)}")
        print("--------------------------------")

if __name__ == '__main__':
    total_bytes = 0
    last_time = time.time()
    stream_url = "rtsp://172.26.108.90:8554/test/rgb"
    main(stream_url)

import gi
import sys
import time
import threading
import psutil
import statistics

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# === Configuration Parameters ===
VIDEO_SOURCE_URL = "rtsp://192.168.144.25:8554/video1"
VIDEO_SINK_URL = "rtsp://orin-nano-10:8554/test/rgb"
SOURCE_BUFFER_MS = 0
SINK_BUFFER_MS = 0
DROP_ON_LATENCY = True
CODEC = 1  # 1 = H264, 2 = H265
CPU_LOG_INTERVAL = 0.5 # seconds

CODEC_NAMES = {1: "H264", 2: "H265"}

from siyi_sdk.siyi_sdk import SIYISDK

def get_codec():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)

    cam.requestGimbalCameraCodecSpecs(1)
    time.sleep(0.1)
    results = cam.getGimbalCameraCodecSpecs()
    print("Codec Specs:", results)
    time.sleep(1.0)
    cam.disconnect()

    codec_type = CODEC_NAMES.get(results.get("video_enc_type", 1), "H264")
    width = results.get("resolution_l", 1920)
    height = results.get("resolution_h", 1080)
    bitrate = results.get("video_bitrate", 4000)
    return codec_type, width, height, bitrate

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

# === CPU Logger ===
def cpu_logger(stop_event, interval, logfile, samples):
    process = psutil.Process()

    # Create the file with placeholders for the summary + header
    with open(logfile, "w") as f:
        f.write("# Average CPU: 0.00%\n")
        f.write("# Std Dev CPU: 0.00%\n")
        f.write("timestamp, cpu_percent\n")
        f.flush()

    # Now reopen in append mode for logging, but update summary separately
    with open(logfile, "a") as f:
        while not stop_event.is_set():
            cpu_percent = process.cpu_percent(interval=None)
            ts = time.time()
            samples.append(cpu_percent)

            # Write the new log line
            line = f"{ts:.2f}, {cpu_percent:.3f}\n"
            f.write(line)
            f.flush()

            # Compute running stats
            avg_cpu = statistics.mean(samples)
            std_cpu = statistics.stdev(samples) if len(samples) > 1 else 0.0

            # Update top summary lines in-place
            with open(logfile, "r+") as updater:
                lines = updater.readlines()
                lines[0] = f"# Average CPU: {avg_cpu:.2f}%\n"
                lines[1] = f"# Std Dev CPU: {std_cpu:.2f}%\n"
                updater.seek(0)
                updater.writelines(lines)
                updater.truncate()

            stop_event.wait(interval)

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
    rtspsrc.set_property("protocols", "tcp")

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

    # Start CPU logger thread
    stop_event = threading.Event()
    samples = []
    codec_type, width, height, bitrate = get_codec()
    log_file_name = f"compute_logs/simple_{codec_type.lower()}_{width}x{height}_{bitrate}_cpu_usage.log"
    logger_thread = threading.Thread(target=cpu_logger, args=(stop_event, CPU_LOG_INTERVAL, log_file_name, samples))
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

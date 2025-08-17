import gi
import time
import psutil
import statistics
import sys
import os

gi.require_version('Gst', '1.0')
from gi.repository import Gst
sys.path.append(os.path.join(os.path.dirname(__file__), '..','gimbal', 'siyi_sdk'))

CPU_LOG_INTERVAL = 0.5 # secondsimport sys

CODEC_NAMES = {1: "H264", 2: "H265"}

from siyi_sdk import SIYISDK

def get_codec():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)

    cam.requestGimbalCameraCodecSpecs(1)
    time.sleep(0.1)
    results = cam.getGimbalCameraCodecSpecs()
    _, video_enc_type, resolution_l, resolution_h, video_bitrate = results
    print("Codec Specs:", results)
    time.sleep(1.0)
    cam.disconnect()

    codec_type = CODEC_NAMES[video_enc_type]

    return codec_type, resolution_l, resolution_h, video_bitrate

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

# === CPU Logger ===
def cpu_logger(stop_event, logfile, samples):
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

            stop_event.wait(CPU_LOG_INTERVAL)
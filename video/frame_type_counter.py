import av
import time

I_FRAME = 1
P_FRAME = 2
B_FRAME = 3

def save_results_to_file(filename, percentages, counts, freqs, periods):
    with open(filename, 'w') as f:
        f.write("Frame Type Percentages (%):\n")
        for k, v in percentages.items():
            f.write(f"  {k}: {v:.2f}%\n")
        f.write("\nFrame Type Counts:\n")
        for k, v in counts.items():
            f.write(f"  {k}: {v}\n")
        f.write("\nFrame Type Frequencies (frames/sec):\n")
        for k, v in freqs.items():
            f.write(f"  {k}: {v:.3f}\n")
        f.write("\nFrame Type Periods (sec/frame):\n")
        for k, v in periods.items():
            f.write(f"  {k}: {v:.3f}\n")

def get_frame_type_percentage(rtsp_url, max_time=10):
    container = av.open(rtsp_url)
    stream = container.streams.video[0]

    frame_counts = {'I': 0, 'P': 0, 'B': 0}
    total_frames = 0

    start_time = time.time()
    time_exceeded = False

    for packet in container.demux(stream):
        for frame in packet.decode():
            frame_type = frame.pict_type

            if frame.key_frame and frame_type == I_FRAME:
                frame_type_str = 'I'
            elif frame_type == P_FRAME:
                frame_type_str = 'P'
            elif frame_type == B_FRAME:
                frame_type_str = 'B'
            else:
                print(f"Unknown frame type: {frame_type}")
                continue

            frame_counts[frame_type_str] += 1
            total_frames += 1

            if time.time() - start_time > max_time:
                time_exceeded = True
                break 

        if time_exceeded:
            break  

    percentages = {k: (v / total_frames) * 100 for k, v in frame_counts.items()} if total_frames > 0 else {}
    frequencies = {k: (v / max_time) for k, v in frame_counts.items()} if max_time > 0 else {}
    periods = {k: (1 / freq) if freq > 0 else 0 for k, freq in frequencies.items()}
    container.close()

    return percentages, frame_counts, frequencies, periods

# Example usage
rtsp_url = "rtsp://192.168.144.25:8554/video1"
percentages, counts, freqs, periods = get_frame_type_percentage(rtsp_url)
print("Frame type percentages:", percentages)
print("Frame type counts:", counts)
print("Frame type frequencies:", freqs)
print("Frame type periods:", periods)

save_results_to_file("frame_analysis_results.txt", percentages, counts, freqs, periods)


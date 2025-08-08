import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

i_frames = 0
p_frames = 0
b_frames = 0

def parse_nalu_type(buffer):
    data = buffer.extract_dup(0, buffer.get_size())
    nalu_type = data[0] & 0x1F  # For H.264
    return nalu_type

def on_buffer_probe(pad, info):
    global i_frames, p_frames, b_frames
    buffer = info.get_buffer()
    if not buffer:
        return Gst.PadProbeReturn.OK

    nalu_type = parse_nalu_type(buffer)
    if nalu_type == 5:
        i_frames += 1
    elif nalu_type in (1, 2):
        p_frames += 1
    elif nalu_type in (0, 3, 4):  # rough approx for B frames; may need deeper parsing
        b_frames += 1

    return Gst.PadProbeReturn.OK

pipeline = Gst.parse_launch(
    "filesrc location=video.mp4 ! qtdemux ! h264parse ! fakesink name=sink"
)

sink = pipeline.get_by_name("sink")
pad = sink.get_static_pad("sink")
pad.add_probe(Gst.PadProbeType.BUFFER, on_buffer_probe)

pipeline.set_state(Gst.State.PLAYING)

loop = GLib.MainLoop()
try:
    loop.run()
except KeyboardInterrupt:
    pipeline.set_state(Gst.State.NULL)
    print(f"I-frames: {i_frames}")
    print(f"P-frames: {p_frames}")
    print(f"B-frames: {b_frames}")

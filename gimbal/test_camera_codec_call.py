import time
import sys
import os
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
siyi_sdk_path = os.path.join(parent_directory, "siyi_sdk")
  
sys.path.append(parent_directory)
sys.path.append(siyi_sdk_path)

from siyi_sdk.siyi_sdk import SIYISDK

def test_codec_reset():
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("No connection ")
        exit(1)

    cam.sendGimbalCameraCodecSpecs(stream_type=1, video_enc_type=1, resolution_l=1920, resolution_h=1080, video_bitrate=4000)
    time.sleep(0.1)
    cam.requestGimbalCameraCodecSpecs(1)
    time.sleep(0.1)
    results = cam.getGimbalCameraCodecSpecs()
    print("Codec Specs:", results)
    time.sleep(1.0)
    cam.disconnect()
    
if __name__ == "__main__":
    test_codec_reset()
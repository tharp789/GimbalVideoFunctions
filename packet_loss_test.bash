#!/bin/bash

LOSS_LEVELS=(0 5 10 25 50)
DEVICE=lo  # change to your interface (eth0, etc)

for LOSS in "${LOSS_LEVELS[@]}"; do
  echo "Testing with $LOSS% packet loss..."

  # Set packet loss
  sudo tc qdisc add dev $DEVICE root netem loss ${LOSS}%

  # Record video for 3 seconds
  gst-launch-1.0 -e \
    udpsrc port=5000 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! \
    rtph264depay ! h264parse ! mp4mux ! filesink location=output_loss_${LOSS}.mp4 &

  RECEIVER_PID=$!
  sleep 3
  kill $RECEIVER_PID

  # Remove packet loss
  sudo tc qdisc del dev $DEVICE root

  echo "Done with $LOSS%"
done

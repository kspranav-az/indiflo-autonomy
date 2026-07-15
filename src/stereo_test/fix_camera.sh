#!/bin/bash
# Hard-recover nvarguscamerasrc without rebooting

echo "=== Argus camera hard recovery ==="

# 1. Nuke every Argus/GStreamer process
sudo killall -9 nvargus-daemon      2>/dev/null
sudo killall -9 nvarguscamerasrc    2>/dev/null
sudo killall -9 gst-launch-1.0      2>/dev/null
sudo killall -9 nvgstcapture-1.0    2>/dev/null
sleep 1

# 2. If systemd manages it, force restart
sudo systemctl stop nvargus-daemon  2>/dev/null
sleep 1
sudo systemctl start nvargus-daemon 2>/dev/null
sleep 1

# 3. Fallback manual start
if ! pgrep -x "nvargus-daemon" > /dev/null; then
    echo "Starting nvargus-daemon manually..."
    sudo /usr/sbin/nvargus-daemon >/dev/null 2>&1 &
    sleep 2
fi

# 4. Status
if pgrep -x "nvargus-daemon" > /dev/null; then
    echo "[OK] nvargus-daemon is running."
else
    echo "[FAIL] nvargus-daemon could not be restarted. Reboot required."
fi

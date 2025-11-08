# VectorNav (VN-100) — Quick Setup

1) Find device
- Plug IMU and check:
```
dmesg | tail -n 20
ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id
```

2) Inspect attributes
```
udevadm info --attribute-walk --name=/dev/ttyUSB0
```

3) udev rule (replace XXXX/ YYYY/ ZZZZ)
```
# udev/rules.d/99-vn100.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", ATTRS{serial}=="ZZZZ", SYMLINK+="imu", MODE="0666"
```

4) Install and reload
```
sudo cp udev/99-vn100.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

5) Docker (optional)
```
--privileged -v /dev/imu:/dev/imu
```

6) Launch ROS 2
```
ros2 launch vectornav vectornav.launch.py
```

# insta360_usb_cam

## udev rule

By default, the launch file uses `/dev/video-insta360` if it exists. Otherwise it finds the Insta360 Link capture node (index 0) automatically. The udev alias below is optional.

```bash
sudo nano /etc/udev/rules.d/99-insta360.rules
```

```udev
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2e1a", ATTRS{idProduct}=="4c01", ATTR{index}=="0", SYMLINK+="video-insta360"
```

`ATTR{index}=="0"` selects the actual capture node among the multiple `/dev/videoN` nodes registered by UVC.

Apply:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify:

```bash
ls -l /dev/video-insta360
```

## Build

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select insta360_usb_cam
source install/setup.bash
```

## Run

```bash
ros2 launch insta360_usb_cam usb_cam.launch.py
```

To select a device explicitly, use `device:=/dev/videoN`. The launch fails with an error if that path does not exist or automatic detection does not find exactly one capture node.

## 수정부분

자동 탐색과 device:=/dev/videoN 직접 지정 방법

## Camera pan center and lock

`config/camera_config.yaml` sets `pan: 0` as the nominal center at startup and `pan_locked: true` to ignore `/camera1/pan_tilt` pan commands and reject runtime `pan` parameter changes. Tilt remains adjustable. To fine-tune the mounted camera heading, change `pan` in the YAML and restart the camera node. Verify the center visually: the actual motor position and any camera-internal tracking mode are not measured by this configuration.

# Custom Camera Stack Installation Guide (Pi 5 / Ubuntu)

This package contains custom-built versions of libcamera and rpicam-apps optimized for the Raspberry Pi 5 running Ubuntu. It specifically enables the imx219 (and other standard Pi cameras) without requiring a full source compilation on every new machine.
📦 Files Required

Ensure you have these two .deb files in your current folder:

    my-libcamera-custom_1.0-1_arm64.deb

    my-rpicam-apps_1.0-1_arm64.deb

## 🚀 Installation Steps

1. Install System Dependencies

Your custom packages contain the camera software, but they rely on standard system libraries (Qt, FFmpeg, Boost) to run. Install them first:
```Bash
sudo apt update
sudo apt install -y \
  libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev \
  libdrm-dev libexif-dev libboost-dev libboost-program-options-dev \
  libjpeg-dev libpng-dev libtiff-dev \
  libepoxy-dev libgl-dev \
  qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
```

2. Install the Custom Packages

Order is important. You must install the library before the applications.
```Bash
# 1. Install libcamera (The core library)
sudo dpkg -i my-libcamera-custom_1.0-1_arm64.deb

# 2. Install rpicam-apps (The tools like rpicam-hello)
sudo dpkg -i my-rpicam-apps_1.0-1_arm64.deb
```

Note: If `dpkg` complains about missing dependencies, run `sudo apt --fix-broken install` and try again.

3. Update Linker & Permissions

Configure the system to find the new libraries and give your user permission to access the camera hardware.
```Bash
# Refresh the system library cache
sudo ldconfig

# Add current user to video and render groups (Required for no-sudo access)
sudo usermod -a -G render,video $USER
```

4. Reboot

You must reboot (or log out and back in) for the user permission changes to take effect.
```Bash
sudo reboot
```

## ✅ Verification

After rebooting, verify the installation.

1. Check if the camera is detected:
```Bash
rpicam-hello --list-cameras
```

Output should show `0 : imx219 ...`

2. Test the Preview Window:
```Bash
rpicam-hello
```

3. Take a Test Photo:
```Bash
rpicam-still -o test.jpg
```

### 🛠 Troubleshooting

Error: "Could not open any dmaHeap device" / "failed to allocate capture buffers" If you still see this after adding permissions and rebooting, you may need to force a larger CMA (memory) allocation in `/boot/firmware/config.txt`:
```Plaintext
dtoverlay=vc4-kms-v3d,cma-512
```
Error: "error while loading shared libraries" If the app complains it can't find libraries, ensure you ran the linker update:
```Bash
sudo ldconfig
```

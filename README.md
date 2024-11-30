# Focal-systems.Corp FT9201Fingerprint.̚ linux driver

Device id `2808:93a9`.

Based on [libfprint wiki post](https://gitlab.freedesktop.org/libfprint/wiki/-/wikis/Devices/2808:93a9)

Not yet production ready but it's good enough for trying out. You might encounter occasional kernel oops or panic.

**Note:** It won't work with User login settings as libfprint integration is not done yet. However, it enables utilization of the fingerprint reader in custom programs.

## Pre-Req
Run following and verify that you have **2808:93a9 Focal-systems.Corp FT9201Fingerprint.̚**
```shell
lsusb
```


# Installation

1. Clone the repo.
2. Get into the project directory: `cd ft9201-fingerprint-driver/`
3. Compile the driver: `make`
4. Either disable **Secure Boot** or follow below steps to sign and enroll MOK key:
    - Generate Private key and Certificate:
      ```shell
      openssl req -new -x509 -newkey rsa:2048 -keyout MOK.priv -outform DER -out MOK.der -nodes -days 36500 -subj "/CN=Custom Kernel Module Signing/"
      ```
    - Sign the compiled driver (kernel module):
      ```shell
      sudo /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 ./MOK.priv MOK.der ft9201.ko
      ```
    - Enroll MOK key: `sudo mokutil --import MOK.der`
    - Reboot and verify MOK key at boot.
5. Install the module in System: `sudo make install`
6. Update module dependencies: `sudo depmod -a`
7. Load the kernel module (driver): `sudo modprobe ft9201`

If you don't want to install the driver, you can load it manually with
```shell
insmod ./ft9201.ko
```

# Usage

If driver is installed, it will autoload when appropriate device is connected.

Once the driver is loaded, and a device is present, it will be available at `/dev/fpreader0`.
  
1. Get into the project directory: `cd ft9201-fingerprint-driver/`
2. Check and verify module name: `sudo ls -lah /dev/fpreader*`
3. Let's say if module name is **fpreader4**, initialize the driver with: `sudo ./ft9201_util /dev/fpreader4`
   
   **Sample output:**
   ```shell
    FT9201 utility program
    Action: 0
    Device: /dev/fpreader4
    status struct: 0x7ffcc1ff7360
    Initializing device
    Setting auto power
   ```
  
 5. Run `sudo cat /dev/fpreader4 > fingerprint.rawimg` and swipe your finger on the scanner so that utility can save image of your fingerprint.
 6. Import raw image in GIMP as RGB,64x80 or convert using imagemagick:
    ```shell
    convert -size 64x80 -depth 8 gray:./fingerprint.rawimg fingerprint.png
    ```




## Utility

```shell
make ft9201_util
```

# Notes

* If something happens during initialization and driver stops sending images, you need to plug it into a windows machine
which will reset it into a stable state. This is being worked on.
* libfprint integration is planned

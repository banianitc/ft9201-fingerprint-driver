# Focal-systems.Corp FT9201Fingerprint.̚ linux driver

Device id `2808:93a9`.

Based on [libfprint wiki post](https://gitlab.freedesktop.org/libfprint/wiki/-/wikis/Devices/2808:93a9)

Not yet production ready but it's good enough for trying out. You might encounter occasional kernel oops or panic.

# Usage

If driver is installed, it will autoload when appropriate device is connected.

Once the driver is loaded, and a device is present, it will be available at `/dev/fpreader0`.

1. Initialize the driver with `./ft9201_util /dev/fpreader0`
2. Capture a fingerprint: `cat /dev/fpreader0 > fingeprint.rawimg`
3. Convert raw image data into png with imagemagick: `convert -size 64x80 -depth 8 gray:./fingerprint.rawimg fingerprint.png`

# Installation

## Driver

```shell
make
make install
```

If you don't want to install the driver, you can load it manually with
```shell
insmod ./ft9201.ko
```

## Utility

```shell
make ft9201_util
```

# Notes

* If something happens during initialization and driver stops sending images, you need to plug it into a windows machine
which will reset it into a stable state. This is being worked on.
* libfprint integration is planned
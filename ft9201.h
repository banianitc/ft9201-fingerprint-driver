#pragma once

#include <linux/ioctl.h>

#define 	FT9201_MAGIC 'F'

#define 	FT9201_IOCTL_REQ_INITIALIZE 		_IO(FT9201_MAGIC, 0x01)
#define 	FT9201_IOCTL_REQ_GET_STATUS			_IOR(FT9201_MAGIC, 0x02, struct ft9201_status)
#define 	FT9201_IOCTL_REQ_SET_AUTO_POWER		_IO(FT9201_MAGIC, 0x03)
#define 	FT9201_IOCTL_REQ_SENSOR_STATUS 		_IO(FT9201_MAGIC, 0x04)

struct ft9201_status {
	unsigned int initialized;
	unsigned short sui_version;
	unsigned char sensor_mcu_state;
	int chip_variant;
	unsigned char sensor_width;
	unsigned char sensor_height;
	unsigned short afe_chip_id;
	unsigned char fw_version;
	unsigned char agc_version;


	unsigned char *raw_image_dta;
};
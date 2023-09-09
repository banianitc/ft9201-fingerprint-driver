#include <linux/module.h>
#include <linux/printk.h>
#include <linux/usb.h>

#include "./ft9201.h"

MODULE_AUTHOR("Mak Krnic <mak@banianitc.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("FT9201 Fingeprint reader driver");

#define VENDOR_ID 0x2808
#define PRODUCT_ID 0x93a9

static struct usb_device_id ft9201_table[] = {
		{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
		{}
};
MODULE_DEVICE_TABLE(usb, ft9201_table);

#define FT9201_MINOR_BASE	192

#define WRITES_IN_FLIGHT	8

#define USB_CONTROL_OP_TIMEOUT 1000
#define USB_READ_OP_TIMEOUT 1000

struct ft9201_device {
	struct usb_device *udev;
	struct usb_interface *interface;
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	int			errors;			/* the last request tanked */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	unsigned long		disconnected:1;
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */

	struct ft9201_status device_status;

	bool            ongoing_read;           /* a read is going on */
	unsigned char   *read_img_data;
	size_t			img_in_size;		/* the size of the receive buffer */
	size_t			img_in_filled;		/* number of bytes in the buffer */
	size_t			img_in_copied;		/* already copied to user space */

};
#define to_ft9201_dev(d) container_of(d, struct ft9201_device, kref)

static int ft9201_open(struct inode *inode, struct file *file);
static int ft9201_release(struct inode *inode, struct file *file);
static long ft9201_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t ft9201_read(struct file *fp, char __user *buf, size_t count, loff_t *f_pos);

static int ft9201_prepare_for_mcu_status_check(struct ft9201_device *dev);

static int ft9201_initialize(struct ft9201_device *dev);
static long ft9201_ioctl_get_status(struct ft9201_device *dev, struct ft9201_status *device_status);
static int ft9201_get_sui_version(struct ft9201_device *dev, unsigned short* version);
static int ft9201_get_afe_state(struct ft9201_device *dev, unsigned short index, unsigned char* value);
static int ft9201_get_sensor_mcu_states(struct ft9201_device *dev, unsigned char *states);
static int ft9201_ic_sensor_mode_exit(struct ft9201_device *dev);
static int ft9201_init_auto_power(struct ft9201_device *dev);
static int ft9201_probably_start_capture(struct ft9201_device *dev, unsigned short value);
static int ft9201_set_afe_state(struct ft9201_device *dev, unsigned short index, unsigned short value);

static int ft9201_read_sensor_dimensions(struct ft9201_device * dev);
static int ft9201_read_image(struct ft9201_device *dev);
static int ft9201_load_afe_chip_id(struct ft9201_device *dev);

static void ft9201_delete(struct kref *kref);

static struct usb_driver ft9201_driver;

static const struct file_operations ft9201_fops = {
		.owner =   THIS_MODULE,
		.llseek =  noop_llseek,
		.open =    ft9201_open,
		.release = ft9201_release,
		.unlocked_ioctl = ft9201_ioctl,
		.read =    ft9201_read,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver ft9201_class = {
		.name =	        "fpreader%d",
		.fops =	        &ft9201_fops,
		.minor_base =   FT9201_MINOR_BASE,
};

// IN Requests
#define FT9201_REQ_GET_SENSOR_INT_PORT_STATES 0x43
#define FT9201_REQ_READ_REGISTERS 0x3a
#define FT9201_REQ_GET_SUI_VERSION 0x1a

// OUT Requests
#define FT9201_REQ_START_CAPTURE_PROBABLY 0x34
#define FT9201_REQ_CONFIGURE_BULK_TRANSFER_SIZE_PROBABLY 0x35
#define FT9201_REQ_WRITE_REGISTER 0x3b

#define FT9201_REG_MCU_SENSOR_STATUS_INDEX 0x20

#define FT9201_AFE_0X30_SUCCESSFUL_RESPONSE 0xbb

static long ft9201_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long errCode;
	struct ft9201_status device_status;
	struct ft9201_device *dev = file->private_data;
	struct ft9201_status *resp = (struct ft9201_status*) arg;

	pr_info("ft9201 ioctl, cmd: %u\n", cmd);

	errCode = 0;

	switch (cmd) {
		case FT9201_IOCTL_REQ_INITIALIZE:
			errCode = ft9201_initialize(dev);
			if (errCode < 0) {
				dev_err(&dev->interface->dev, "Error initializing device: %ld", errCode);
				return errCode;
			}
			break;
		case FT9201_IOCTL_REQ_GET_STATUS:
			if ((void*)arg == NULL) {
				return -EFAULT;
			}

			errCode = ft9201_ioctl_get_status(dev, &device_status);
			if (errCode < 0) {
				dev_err(&dev->interface->dev, "Error getting device status: %ld", errCode);
				return errCode;
			}

			pr_info("device status: %p, %d", &device_status, device_status.sui_version);
			errCode = copy_to_user(resp, &device_status, sizeof(device_status));
			if (errCode) {
				dev_err(&dev->interface->dev, "Error copying data to user: %ld", errCode);
				return errCode;
			}
			break;

		case FT9201_IOCTL_REQ_SET_AUTO_POWER:
			errCode = ft9201_init_auto_power(dev);
			if (errCode < 0) {
				dev_err(&dev->interface->dev, "Error setting auto power: %ld", errCode);
				return errCode;
			}
			break;

		case FT9201_IOCTL_REQ_SENSOR_STATUS:
			dev_info(&dev->interface->dev, "sensor status");
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static int ft9201_get_sensor_int_port_states(struct ft9201_device *dev, unsigned char *states)
{
	int retval;
	unsigned char local_states;;
	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_GET_SENSOR_INT_PORT_STATES,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			0,
			&local_states,
			sizeof(local_states),
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}

	dev_info(&dev->interface->dev, "Read states: 0x%02x", local_states);
	*states = local_states;

	return retval;
}

static int ft9201_is_data_present(struct usb_device *udev, unsigned char *present)
{
	int retval;
	retval = usb_control_msg_recv(
			udev,
			0,
			FT9201_REQ_GET_SENSOR_INT_PORT_STATES,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			0,
			present,
			1,
			5000,
			GFP_KERNEL);

	if (retval) {
		dev_info(&udev->dev, "Error sending data: %d\n", retval);
		return retval;
	}
	dev_info(&udev->dev, "Sent something");

	return retval;
}

static int ft9201_prepare_for_mcu_status_check(struct ft9201_device *dev)
{
	int retval;
	struct usb_device *udev = dev->udev;

	int i;

	for (i = 0; i < 2; i++) {
		retval = usb_control_msg_send(
				udev,
				0,
				0x22,
				USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				0x70,
				0x70,
				NULL,
				0,
				5000,
				GFP_KERNEL);

		if (retval) {
			dev_info(&udev->dev, "Error sending data: %d\n", retval);
			return retval;
		}

		msleep(20);
	}

	dev_info(&udev->dev, "Prepare for mcu status check done");

	return retval;
}

static int ft9201_get_sui_version(struct ft9201_device *dev, unsigned short* version)
{
	// For some reason, Windows driver reads 4 bytes (2x ushort), and then discards the second short
	unsigned short sui_version[2];
	int retval;
	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_GET_SUI_VERSION,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			0,
			sui_version,
			sizeof(sui_version),
			5000,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}
	dev_info(&dev->interface->dev, "Sent something");
	dev_info(&dev->interface->dev, "sui version: %d", sui_version[0]);
	*version = sui_version[0];

	return retval;
}

static int ft9201_get_afe_state(struct ft9201_device *dev, unsigned short index, unsigned char* value)
{
	unsigned char local_value[4];
	int retval;
	index = index & 0xFF;
	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_READ_REGISTERS,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			index,
			&local_value,
			sizeof(local_value),
			5000,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}
	dev_info(&dev->interface->dev, "Read AFE at index 0x%02x: 0x%02x%02x%02x%02x", index, local_value[0], local_value[1], local_value[2], local_value[3] );
	if (value == NULL) {
		return -EFAULT;
	}
	*value = local_value[0];

	return retval;
}

static int ft9201_get_sensor_mcu_states(struct ft9201_device *dev, unsigned char *states)
{
	// There seems to be a correlation between last scanned fingerprint position and the values
	// this call receives. In general, it seems that response is always 0x0000xyxy.
	// It keeps receiving the same data until next fingerprint is scanned.

	unsigned char buffer[4];

	int retval;

	if (states == NULL) {
		return -EFAULT;
	}

	retval = usb_control_msg_recv(
			dev->udev,
			0,
			FT9201_REQ_READ_REGISTERS,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			FT9201_REG_MCU_SENSOR_STATUS_INDEX,
			&buffer,
			sizeof(buffer),
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}

	if (buffer[0] == 0xa5 && buffer[1] == 0x5a) {
		*states = 1;
	} else {
		*states = 0;
	}

	return retval;
}

static int ft9201_set_afe_state(struct ft9201_device *dev, unsigned short index, unsigned short value)
{
	int retval;
	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_WRITE_REGISTER,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value & 0xff,
			index,
			NULL,
			0,
			USB_CONTROL_OP_TIMEOUT,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}

	return retval;
}

static int ft9201_probably_start_capture(struct ft9201_device *dev, unsigned short value)
{
	int retval;
	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_START_CAPTURE_PROBABLY,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value & 0xff,
			0,
			NULL,
			0,
			5000,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}

	return retval;
}

static int ft9201_probably_configure_bulk_transfer(struct ft9201_device *dev, unsigned short index, unsigned short value)
{
	int retval;
	retval = usb_control_msg_send(
			dev->udev,
			0,
			FT9201_REQ_CONFIGURE_BULK_TRANSFER_SIZE_PROBABLY,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value,
			index,
			NULL,
			0,
			5000,
			GFP_KERNEL);

	if (retval) {
		dev_info(&dev->interface->dev, "Error sending data: %d\n", retval);
		return retval;
	}

	return retval;
}

#define AUTO_POWER_MAX_RETRY_COUNT 5

static int ft9201_init_auto_power(struct ft9201_device *dev)
{
	int errCode = 0;
	unsigned char sensor_mcu_state;
	int i;

	for (i = 0; i < AUTO_POWER_MAX_RETRY_COUNT; i++) {
		errCode = ft9201_get_sensor_mcu_states(dev,&sensor_mcu_state);
		if (errCode < 0) {
			return errCode;
		}

		if (sensor_mcu_state == 1) {
			break;
		}

		// Let's wait for mcu to power up
		msleep(10);
	}

	errCode = ft9201_set_afe_state(dev, 0x1f, 1);
	if (errCode < 0) {
		dev_err(&dev->interface->dev, "Error setting power something 1 to 0x1f: Error %d", errCode);
		return errCode;
	}

	errCode = ft9201_set_afe_state(dev, 0x1e, 1);
	if (errCode < 0) {
		dev_err(&dev->interface->dev, "Error setting power something 1 to 0x1e: Error %d", errCode);
		return errCode;
	}

	msleep(10);

	for (i = 0; i < AUTO_POWER_MAX_RETRY_COUNT; i++) {
		sensor_mcu_state = 0;

		errCode = ft9201_get_sensor_mcu_states(dev,&sensor_mcu_state);
		if (errCode < 0) {
			return errCode;
		}

		if (sensor_mcu_state == 1) {
			break;
		}

		msleep(1);
	}

	errCode = ft9201_get_afe_state(dev, 0x1d, &sensor_mcu_state);
	if (errCode) {
		return errCode;
	}
	sensor_mcu_state = 0;
	errCode = ft9201_get_sensor_mcu_states(dev,&sensor_mcu_state);
	if (errCode < 0) {
		return errCode;
	}
	// supposedly we're good now

	return errCode;
}

static int ft9201_open(struct inode *inode, struct file *file)
{
	struct usb_interface *intf;
	struct ft9201_device *dev;

	pr_info("ft9201 open\n");

	intf = usb_find_interface(&ft9201_driver, iminor(inode));
	if (!intf) {
		pr_err("Can't find device for minor %d\n", iminor(inode));
		return -ENODEV;
	}

	dev = usb_get_intfdata(intf);
	if (!dev) {
		return -ENODEV;
	}

	kref_get(&dev->kref);

	file->private_data = dev;

	return 0;
}

static int ft9201_release(struct inode *inode, struct file *file)
{
	struct ft9201_device *dev = file->private_data;
	pr_info("ft9201 release\n");

	if (dev == NULL) {
		return -ENODEV;
	}

	kref_put(&dev->kref, ft9201_delete);

	return 0;
}

static int ft9201_read_sensor_dimensions(struct ft9201_device * dev)
{
	int ret = 0;
	ret = ft9201_get_afe_state(dev, 0x14, &dev->device_status.sensor_width);
	if (ret) {
		return ret;
	}
	return ft9201_get_afe_state(dev, 0x15, &dev->device_status.sensor_height);
}

static int ft9201_load_afe_chip_id(struct ft9201_device *dev)
{
	int ret;
	unsigned char chip_id_high;
	unsigned char chip_id_low;

	ret = ft9201_get_afe_state(dev, 0x16, &chip_id_high);
	if (ret < 0) {
		return ret;
	}
	ret = ft9201_get_afe_state(dev, 0x17, &chip_id_low);
	if (ret < 0) {
		return ret;
	}

	dev->device_status.afe_chip_id = ((chip_id_high & 0xff) << 8) | (chip_id_low & 0xff);
	dev_info(&dev->interface->dev, "afe sum: %04x\n", dev->device_status.afe_chip_id);

	return ret;
}

static int ft9201_initialize(struct ft9201_device *dev)
{
	int errCode;
	unsigned char sensor_status;
	char *chip_variant_str;

	dev_info(&dev->interface->dev, "ioctl initialize");

	// errCode = ft9201_prepare_for_mcu_status_check(dev);
	errCode = ft9201_get_sensor_mcu_states(dev, &sensor_status);
	if (errCode) {
		dev_err(&dev->interface->dev, "Error getting sensor status");
		return errCode;
	}

	if (sensor_status != 1) {
		ft9201_ic_sensor_mode_exit(dev);
	}

	ft9201_load_afe_chip_id(dev);

	switch (dev->device_status.afe_chip_id) {
		case 0x9338: // -0x6cc8
			// some more AFE reading
			break;
		case 0x9536: // -0x6aca
			// some more AFE reading
			break;
		case 0x95a8: // -0x6a58
			errCode = ft9201_read_sensor_dimensions(dev);
			if (errCode) {
				dev_err(&dev->interface->dev, "Error reading sensor dimensions: %d", errCode);
				return errCode;
			}

			dev_info(&dev->interface->dev, "Image dimensions: %d x %d", dev->device_status.sensor_width, dev->device_status.sensor_height);

			if (dev->device_status.sensor_height != 0x60 || dev->device_status.sensor_width != 0x60) {
				dev->device_status.chip_variant = 3;
			}
			break;
	}


	switch(dev->device_status.chip_variant) {
		case 1:
			chip_variant_str = "FT9338W";
			break;
		case 2:
			chip_variant_str = "FT9 348w";
			break;
		case 3:
			chip_variant_str = "FT9361";
			break;
		case 6:
			chip_variant_str = "FT9536w";
			break;
		default:
			chip_variant_str = "UNKNOWN";
	}

	dev_info(&dev->interface->dev, "AFE Chip ID = 0x%x, FT93xx = %s (%d)", dev->device_status.afe_chip_id, chip_variant_str, dev->device_status.chip_variant);

	// Here goes firmware download

	// All of this is happy path. Need to implement all error handling.

	// This block pretends to be firmware download internal
	{
		// if afe 0x30 response == 0xbb, then nothing
		// else
		//   We call SetDeviceConfig with some parameters that seem to be ignored.
		//   It then sets some power settings including setting 0xbb to index 0x30
		//


		ft9201_get_afe_state(dev, 0x1a, &dev->device_status.fw_version);
		// then ignore this

		ft9201_get_afe_state(dev, 0x3c, &dev->device_status.agc_version);

		dev_info(&dev->interface->dev, "fw version = %d agc version = %d", dev->device_status.fw_version, dev->device_status.agc_version);

		// Some logic for checking whether firmware is latest
		// and goto to actual fw upload code if needed

		errCode = ft9201_read_sensor_dimensions(dev);
		if (errCode) {
			dev_err(&dev->interface->dev, "Error reading sensor dimensions: %d", errCode);
			return errCode;
		}

		if (dev->device_status.chip_variant == 2 || dev->device_status.chip_variant == 3) {
			errCode = ft9201_set_afe_state(dev, 0x22, 0);
			errCode = ft9201_set_afe_state(dev, 0x23, 0xe);
		}

		errCode = ft9201_init_auto_power(dev);
		if (errCode == -1) {
			errCode = ft9201_init_auto_power(dev);
		}
	}

	if (errCode == 0) {
		dev->device_status.initialized = 1;
		dev_info(&dev->interface->dev, "Device initialization successful");
	}
	return errCode;
}

static int ft9201_read_image(struct ft9201_device *dev)
{
	int retVal;
	int img_size = dev->device_status.sensor_width * dev->device_status.sensor_height;
	int transfer_size = img_size + 2;
	int read_length;
	unsigned char *img_with_header;

	dev_info(&dev->interface->dev, "Reading image from scanner; dimensions: %dx%d", dev->device_status.sensor_width, dev->device_status.sensor_height);

	dev->img_in_copied = 0;
	dev->img_in_filled = 0;

	if (dev->read_img_data != NULL) {
		kfree(dev->read_img_data);
		dev->img_in_size = 0;
	}

	dev->read_img_data = kzalloc(img_size, GFP_KERNEL);
	if (dev->read_img_data == NULL) {
		return -ENOMEM;
	}
	dev->img_in_size = img_size;

	img_with_header = kzalloc(transfer_size, GFP_KERNEL);
	if (img_with_header == NULL) {
		retVal = -ENOMEM;
		goto out;
	}

	retVal = usb_bulk_msg(
			dev->udev,
			usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
			img_with_header,
			transfer_size,
			&read_length,
			USB_READ_OP_TIMEOUT
	);
	if (retVal < 0) {
		dev_err(&dev->interface->dev, "Error reading data from device: Error %d", retVal);
		goto out;
	}
	dev_info(&dev->interface->dev, "Received %d bytes from device", read_length);
	if (read_length != transfer_size) {
		dev_err(&dev->interface->dev, "Read less than image size");
		retVal = -EINVAL;
		goto out;
	}

	// Ignore added 2 bytes at the start of data
	memcpy(dev->read_img_data, img_with_header + 2, img_size);
	dev->img_in_filled = img_size;


out:
	if (img_with_header != NULL) {
		kfree(img_with_header);
	}

	return retVal;
}

static int ft9201_ic_sensor_mode_exit(struct ft9201_device *dev)
{
	int errCode;
	unsigned char read_state;

	errCode = ft9201_prepare_for_mcu_status_check(dev);
	if (errCode < 0) {
		return errCode;
	}

	msleep(10);

	errCode = ft9201_get_sensor_mcu_states(dev, &read_state);
	if (errCode < 0) {
		return errCode;
	}


	return errCode;
}

static long ft9201_ioctl_get_status(struct ft9201_device *dev, struct ft9201_status *device_status)
{
	int errCode;

	if (device_status == NULL) {
		return -EFAULT;
	}

	errCode = ft9201_get_sui_version(dev, &device_status->sui_version);
	if (errCode) {
		return errCode;
	}

	errCode = ft9201_get_afe_state(dev, 0x14, &device_status->sensor_width);
	if (errCode) {
		return errCode;
	}

	errCode = ft9201_get_afe_state(dev, 0x15, &device_status->sensor_height);
	if (errCode) {
		return errCode;
	}

	errCode = ft9201_get_sensor_mcu_states(dev,&device_status->sensor_mcu_state);
	if (errCode) {
		return errCode;
	}

	return 0;
}

static DECLARE_WAIT_QUEUE_HEAD(ft9201_wq);

static int has_data_remaining(struct ft9201_device *dev)
{
	dev_info(&dev->interface->dev, "Copied: %lu, Filled: %lu", dev->img_in_copied, dev->img_in_filled);
	return dev->img_in_copied < dev->img_in_filled;
}

static ssize_t send_read_data(struct ft9201_device *dev, char __user *buf, size_t count)
{
	size_t remaining = dev->img_in_filled - dev->img_in_copied;
	size_t to_copy = remaining;
	if (to_copy > count) {
		to_copy = count;
	}
	dev_info(&dev->interface->dev, "Copied: %lu, to_copy: %lu, full_size: %lu", dev->img_in_copied, to_copy, dev->img_in_size);

	if (dev->img_in_copied + to_copy > dev->img_in_size) {
		return -EINVAL;
	}

	if (copy_to_user(buf, dev->read_img_data + dev->img_in_copied, to_copy)) {
		return -EFAULT;
	}
	dev->img_in_copied += to_copy;
	dev_info(&dev->interface->dev, "Copied total: %lu", dev->img_in_copied);
	return (ssize_t)to_copy;
}

static ssize_t ft9201_read(struct file *fp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct ft9201_device *dev = fp->private_data;
	int i;
	unsigned long loop_timeout = msecs_to_jiffies(1000);
	int ret;
	unsigned char data_ready;
	unsigned char translate_data;
	unsigned char afe_0x1d_finger_present;

	if (dev == NULL) {
		pr_err("device is null\n");
		return 0;
	}

	ret = mutex_lock_interruptible(&dev->io_mutex);
	if (ret < 0) {
		pr_info("Interrupted while waiting on IO mutex");
		return ret;
	}

	if (dev->disconnected) {
		ret = -ENODEV;
		goto exit;
	}

	while (true) {
		if (has_data_remaining(dev)) {
			ret = send_read_data(dev, buf, count);
			break;
		}

		pr_info("ft9201 reading %d\n", i);


		ret = wait_event_interruptible_timeout(ft9201_wq, 0, loop_timeout);
		if (ret == -ERESTARTSYS) {
			// We were interrupted by a signal
			pr_info("Interrupted while waiting for read data");
			ret = 0;
			break;
		}

		ret = ft9201_get_sensor_mcu_states(dev, &data_ready);
		if (ret < 0) {
			continue;
		}

		dev_info(&dev->interface->dev, "Data ready: 0x%02x", data_ready);
		if (data_ready) {
			ret = ft9201_get_afe_state(dev, 0x30, &translate_data);
			if (ret) {
				break;
			}
			dev_info(&dev->interface->dev, "0x30 translate data: 0x%02x", translate_data);

			if (translate_data == FT9201_AFE_0X30_SUCCESSFUL_RESPONSE) {
				ret = ft9201_get_afe_state(dev, 0x1d, &afe_0x1d_finger_present);
				if (ret) {
					break;
				}
				dev_info(&dev->interface->dev, "afe 0x1d data: 0x%02x", afe_0x1d_finger_present);

				if (afe_0x1d_finger_present != 1 && afe_0x1d_finger_present != 0xa0) {
					// finger was not detected, retry something
					// maybe we need to return to auto power settings or something similar
				} else {
					dev_info(&dev->interface->dev, "finger detected: 0x%02x", afe_0x1d_finger_present);
					ret = ft9201_probably_start_capture(dev, 0xff);
					if (ret) {
						dev_err(&dev->interface->dev, "error capturing1: %d", ret);
						break;
					}
					msleep(20);

					ret = ft9201_probably_start_capture(dev, 0x3);
					if (ret) {
						dev_err(&dev->interface->dev, "error capturing2: %d", ret);
						break;
					}

					ret = ft9201_probably_configure_bulk_transfer(dev, 0x3400, dev->device_status.sensor_width * dev->device_status.sensor_height + 2);
					if (ret) {
						dev_err(&dev->interface->dev, "error configuring bulk transfer: %d", ret);
						break;
					}


					// Now, after everything is configured for transfer, we need to call read_image,
					// which does bulk in transfer
					ret = ft9201_read_image(dev);
					if (ret < 0) {
						break;
					}
				}
			}
		}
	}

exit:
	mutex_unlock(&dev->io_mutex);
	return ret;
}

static void ft9201_delete(struct kref *kref)
{
	struct ft9201_device *dev = to_ft9201_dev(kref);

	// usb_deregister_dev(dev->interface, &ft9201_class);
	usb_put_intf(dev->interface);
	usb_put_dev(dev->udev);
	if (dev->read_img_data != NULL) {
		kfree(dev->read_img_data);
		dev->read_img_data = NULL;
	}
	kfree(dev);
}

static int ft9201_probe(struct usb_interface *intf, const struct usb_device_id *id) {
	struct usb_device *udev = interface_to_usbdev(intf);
	struct ft9201_device *dev;
	struct usb_endpoint_descriptor *bulk_in, *bulk_out;

	int retval;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
	}

	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	spin_lock_init(&dev->err_lock);
	init_waitqueue_head(&dev->bulk_in_wait);

	dev->udev = usb_get_dev(udev);
	dev->interface = usb_get_intf(intf);

	// bulk_out endpoint is not yet used. It might be used only for upgrading firmware if it's ever implemented
	/* use only the first bulk-in and bulk-out endpoints */
	retval = usb_find_common_endpoints(intf->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL);
	if (retval) {
		dev_err(&intf->dev, "Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	dev->bulk_in_endpointAddr = bulk_in->bEndpointAddress;

	/* save our data pointer in this interface device */
	usb_set_intfdata(intf, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(intf, &ft9201_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&intf->dev,
				"Not able to get a minor for this device.\n");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&intf->dev, "USB fpreader device now attached to fpreader%d", intf->minor);

	retval = ft9201_initialize(dev);
	if (retval < 0) {
		dev_err(&dev->interface->dev, "Error initializing device: %d", retval);
	}

	return 0;

error:
	/* this frees allocated memory */
	kref_put(&dev->kref, ft9201_delete);

	return retval;
}

static int ft9201_pre_reset(struct usb_interface *interface) {
	pr_info("Pre reset\n");

	return 0;
}

static int ft9201_post_reset(struct usb_interface *interface) {
	pr_info("Post reset\n");

	return 0;
}

static void ft9201_disconnect(struct usb_interface *interface) {
	struct ft9201_device *dev;
	int minor = interface->minor;

	pr_info("Disconnect");

	dev = usb_get_intfdata(interface);
	pr_info("Disconnect");
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &ft9201_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);

	/* decrement our usage count */
	kref_put(&dev->kref, ft9201_delete);

	dev_info(&interface->dev, "USB device fpreader%d now disconnected", minor);
}

static int ft9201_suspend(struct usb_interface *intf, pm_message_t message) {
	pr_info("Suspend");

	return 0;
}

static int ft9201_resume(struct usb_interface *intf) {
	pr_info("Resume");

	return 0;
}

static struct usb_driver ft9201_driver = {
		.name = "ft9201",
		.probe = ft9201_probe,
		.disconnect = ft9201_disconnect,
		.suspend = ft9201_suspend,
		.resume = ft9201_resume,
		.pre_reset = ft9201_pre_reset,
		.post_reset = ft9201_post_reset,
		.id_table = ft9201_table,
};

module_usb_driver(ft9201_driver);

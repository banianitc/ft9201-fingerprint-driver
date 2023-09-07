#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>

#include "./ft9201.h"

int main(int argc, char *argv[]) {
	unsigned int action = 0;
	printf("FT9201 utility program\n");

	if (argc < 2) {
		fprintf(stderr, "Usage: %s <device_file> [action]\n", argv[0]);
		return -1;
	}


	if (argc == 3) {
		action = (unsigned int) atoi(argv[2]);
	}

	printf("Action: %d\n", action);
	// if (action > 0x04) {
	// 	fprintf(stderr, "Invalid action: %d\n", action);
	// 	return -1;
	// }




	char *device_file_name = argv[1];
	struct ft9201_status device_status;

	printf("Device: %s\n", device_file_name);
	int fp_reader_fd = open(device_file_name, O_RDWR);
	if (fp_reader_fd < 0) {
		perror("Error opening the device");
		return -1;
	}

	if (action == 0) {
		fprintf(stderr, "status struct: %p\n", &device_status);

		int result;

		printf("Initializing device\n");
		result = ioctl(fp_reader_fd, FT9201_IOCTL_REQ_INITIALIZE, NULL);
		if (result < 0) {
			perror("Error executing ioctl initialize");
			close(fp_reader_fd);
			return -2;
		}

		printf("Setting auto power\n");
		result = ioctl(fp_reader_fd, FT9201_IOCTL_REQ_SET_AUTO_POWER, NULL);
		if (result < 0) {
			perror("Error executing ioctl1");
			close(fp_reader_fd);
			return -2;
		}

		// result = ioctl(fp_reader_fd, FT9201_IOCTL_REQ_GET_STATUS, &device_status);
		// if (result < 0) {
		// 	perror("Error executing ioctl2");
		// 	close(fp_reader_fd);
		// 	return -2;
		// }
	} else if (action == 1) {
		printf("Reading\n");
		fd_set set;
		struct timeval timeout = {
				.tv_sec = 1,
				.tv_usec = 0,
		};
		char buff[100];
		int len = 100;
		int rv;
		FD_ZERO(&set);
		FD_SET(fp_reader_fd, &set);
		rv = select(fp_reader_fd + 1, &set, NULL, NULL, &timeout);
		if (rv == -1) {
			perror("select");
		} else if (rv == 0) {
			fprintf(stderr, "Timeout\n");
		} else {
			fprintf(stderr, "rv: %d\n", rv);
			read(fp_reader_fd, &buff, len);
		}
	}

	close(fp_reader_fd);

	// printf("Device SUI version: 0x%04x\n", device_status.sui_version);
	// printf("Afe state 0x14: 0x%02x%02x%02x%02x\n", device_status.sensor_width[0], device_status.sensor_width[1], device_status.sensor_width[2], device_status.sensor_width[3]);
	// printf("Afe state 0x15: 0x%02x%02x%02x%02x\n", device_status.sensor_height[0], device_status.sensor_height[1], device_status.sensor_height[2], device_status.sensor_height[3]);
	// printf("Sensor MCU state: 0x%02x\n", device_status.sensor_mcu_state);

	return 0;
}
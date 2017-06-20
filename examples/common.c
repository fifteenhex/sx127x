#include <stropts.h>

#include "common.h"
#include "sx127x.h"

#define COLWIDTH 8
#define OFFSET(r, c) ((r * COLWIDTH) + c)
#define REMAINING(r, c, l) ((int) l - OFFSET(r, c))
void printhex(uint8_t* buff, size_t len) {
	int col = 0;
	for (int row = 0; (REMAINING(row, 0, len) > 0); row++) {
		for (col = 0; (col < COLWIDTH) && (REMAINING(row, col, len) > 0);
				col++) {
			uint8_t b = buff[OFFSET(row, col)];
			char c = b >= 0x20 && b <= 0x7e ? b : '.';
			printf("%02x [%c] ", b, c);
		}
		printf("\n");
	}
}

int setupradio(int fd) {
	if (ioctl(fd, SX127X_IOCTL_CMD_SETPAOUTPUT, SX127X_PA_PABOOST) != 0) {
		printf("failed to set pa output\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SETMODULATION, SX127X_MODULATION_LORA)
			!= 0) {
		printf("failed to set modulation\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SETCARRIERFREQUENCY, 920000000) != 0) {
		printf("failed to set carrier frequency\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SETSF, 12) != 0) {
		printf("failed to set spreading factor\n");
		return 1;
	}

	if (ioctl(fd, SX127X_IOCTL_CMD_SETOPMODE, SX127X_OPMODE_RXCONTINUOS) != 0) {
		printf("failed to set opmode\n");
		return 1;
	}
}

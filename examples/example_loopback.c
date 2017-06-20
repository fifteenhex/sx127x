#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stropts.h>

#include "sx127x.h"
#include "common.h"

int main(int argc, char** argv) {

	long count = -1;
	if(argc != 1 && argc != 2){
		printf("usage: %s\n", argv[0]);
		return 1;
	}
	else if(argc == 2){
		count = strtol(argv[1], NULL, 10);
	}

	int fd = open("/dev/sx127x0", O_RDWR);

	if (fd < 0)
		printf("failed to open device\n");

	if(ioctl(fd, SX127X_IOCTL_CMD_SETPAOUTPUT, SX127X_PA_PABOOST) != 0){
                printf("failed to set pa output\n");
                return 1;
        }

	if(ioctl(fd, SX127X_IOCTL_CMD_SETMODULATION, SX127X_MODULATION_LORA) != 0){
		printf("failed to set modulation\n");
		return 1;
	}

	if(ioctl(fd, SX127X_IOCTL_CMD_SETCARRIERFREQUENCY, 920000000)!= 0){
		printf("failed to set carrier frequency\n");
		return 1;
	}

	if(ioctl(fd, SX127X_IOCTL_CMD_SETSF, 12)!= 0){
		printf("failed to set spreading factor\n");
		return 1;
	}

	if(ioctl(fd, SX127X_IOCTL_CMD_SETOPMODE, SX127X_OPMODE_RXCONTINUOS)!= 0){
                printf("failed to set opmode\n");
                return 1;
        }

	void* buff = malloc(1024);

	while(count != 0){
		read(fd, buff, sizeof(size_t));
		read(fd, buff + sizeof(size_t), *((size_t*) buff));

		struct sx127x_pkt* pkt = buff;
		void* payload = buff + pkt->hdrlen;

		printf("payloadlen: %zd bytes, snr: %ddB, rssi: %ddBm\n", pkt->payloadlen,
				(int) pkt->snr, (int) pkt->rssi);
		printhex(payload, pkt->payloadlen);

		write(fd, payload, pkt->payloadlen);
		if(count > 0)
			count--;
	}
	return 0;
}


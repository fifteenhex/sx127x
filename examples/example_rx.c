#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "sx127x.h"

int main(int argc, char** argv) {

	int fd = open("/dev/sx127x0", O_RDWR);

	if (fd < 0)
		printf("failed to open device\n");

	void* buff = malloc(1024);

	read(fd, buff, sizeof(size_t));
	read(fd, buff + sizeof(size_t), *((size_t*) buff));

	struct sx127x_pkt* pkt = buff;
	void* payload = buff + pkt->hdrlen;

	printf("payloadlen: %zd bytes, snr: %ddB, rssi: %ddBm\n", pkt->payloadlen,
			(int) pkt->snr, (int) pkt->rssi);

	return 0;
}


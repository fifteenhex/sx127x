#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "sx127x.h"
#include "common.h"

int main(int argc, char** argv) {

	int fd = open("/dev/sx127x0", O_RDWR);

	if (fd < 0)
		printf("failed to open device\n");
	if (setupradio(fd, SX127X_OPMODE_CAD))
		return 1;

	void* buff = malloc(1024);

	read(fd, buff, sizeof(size_t));
	read(fd, buff + sizeof(size_t), *((size_t*) buff));

	while (1) {

	}

	return 0;
}


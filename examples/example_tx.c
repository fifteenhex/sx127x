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

	if (setupradio(fd))
		return 1;

	char helloworld[] = "hello, world\n";

	write(fd, helloworld, sizeof(helloworld));
	return 0;
}


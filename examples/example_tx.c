#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include "sx127x.h"

int main(int argc, char** argv) {

	int fd = open("/dev/sx127x0", O_RDWR);

	if (fd < 0)
		printf("failed to open device\n");

	char helloworld[] = "hello, world\n";

	write(fd, helloworld, sizeof(helloworld));
	return 0;
}


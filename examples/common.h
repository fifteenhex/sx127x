#pragma once
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include "sx127x.h"

void printhex(uint8_t* buff, size_t len);
int setupradio(int fd, enum sx127x_opmode opmode);

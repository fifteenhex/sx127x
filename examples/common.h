#pragma once
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

void printhex(uint8_t* buff, size_t len);
int setupradio(int fd);

#include "common.h"

#define COLWIDTH 8
#define OFFSET(r, c) ((r * COLWIDTH) + c)
#define REMAINING(r, c, l) ((int) l - OFFSET(r, c))
void printhex(uint8_t* buff, size_t len){
	int col = 0;
	for(int row = 0; (REMAINING(row, 0, len) > 0); row++){
		for(col = 0; (col < COLWIDTH) && (REMAINING(row, col, len) > 0); col++){
			uint8_t b = buff[OFFSET(row, col)];
			char c = b >= 0x20 && b <= 0x7e ? b : '.';
			printf ("%02x [%c] ", b, c);
		}
		printf("\n");
	}
}

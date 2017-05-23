#ifndef SX127X_H_
#define SX127X_H_

#include <linux/types.h>

struct sx127x_pkt {
	size_t len;
	size_t hdrlen;
	size_t payloadlen;

	__s16 snr;
	__s16 rssi;
	__u32 fei;
} __attribute__ ((packed));


#endif /* SX127X_H_ */

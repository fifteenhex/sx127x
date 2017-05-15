#ifndef SX127X_H_
#define SX127X_H_

struct sx127x_pkt {
	size_t hdrlen;
	size_t payloadlen;

	u32 fei;
} __attribute__ ((packed));


#endif /* SX127X_H_ */

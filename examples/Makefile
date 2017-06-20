all: example_tx example_rx example_loopback

example_tx: common.c example_tx.c
	$(CC) -I../ $^ -o $@

example_rx: common.c example_rx.c
	$(CC) -I../ $^ -o $@

example_loopback: common.c example_loopback.c
	$(CC) -I../ $^ -o $@

clean:
	rm -f example_tx example_rx example_loopback

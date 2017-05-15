CONFIG_SX127X := m

sx127x-y := \
	main.o

ldflags-y += --strip-debug

obj-$(CONFIG_SX127X) += sx127x.o


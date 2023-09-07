obj-m += ft9201.o
#CFLAGS := $(CFLAGS) -std=c90

util_objs = util_main.o

PWD:= $(CURDIR)

all: build

ft9201_util: $(util_objs)
	gcc $(util_objs) -o ./ft9201_util

ft9201_util_clean:
	rm $(util_objs)

build: ft9201_util
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean: ft9201_util_clean
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

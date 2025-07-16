# export PATH := $(PATH):$(HOME)/esp/esp-idf/

.PHONY: build flash monit clean config 


TARGET = esp32

config:
	idf.py set-target ${TARGET}
	idf.py menuconfig

build:
	idf.py build

flash:
	idf.py -p /dev/ttyUSB0 flash

monit:
	idf.py monitor

clean:
	rm -rf build/
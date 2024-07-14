
VERSION = $(shell grep Version: mrhat-bq25622/DEBIAN/control | cut -d' ' -f2)
# TODO: build module for all kernel versions
KVER ?= 6.1.21+
TARGET ?=  $(error TARGET not specified for deploy )

all: build/mrhat-bq25622_$(VERSION)-1_armhf.deb
	@true

build/mrhat-bq25622_$(VERSION)-1_armhf.deb :  build/bq2562x_charger.ko build/mrhat-bq25622.dtbo mrhat-bq25622/DEBIAN/*
	mkdir -p build
	mkdir -p mrhat-bq25622/lib/modules/$(KVER)
	mkdir -p mrhat-bq25622/boot/overlays/
	cp build/bq2562x_charger.ko mrhat-bq25622/lib/modules/$(KVER)/
	cp build/mrhat-bq25622.dtbo mrhat-bq25622/boot/overlays/
	dpkg-deb --root-owner-group --build mrhat-bq25622 build/mrhat-bq25622_$(VERSION)-1_armhf.deb

build/bq2562x_charger.ko: driver/*.c driver/*.h
	mkdir -p build
	rsync --delete -r  ./driver/ /tmp/drv-bq25622
	schroot -c buildroot -u root -d /tmp/drv-bq25622 -- make KVER=$(KVER)
	cp /tmp/drv-bq25622/bq2562x_charger.ko build/bq2562x_charger.ko


clean:
	rm -rf mrhat-bq25622/boot/ mrhat-bq25622/lib/ build/

build/mrhat-bq25622.dts.pre: mrhat-bq25622.dts
	mkdir -p build/
	cpp -nostdinc -undef -x assembler-with-cpp -I/var/chroot/buildroot/usr/src/linux-headers-$(KVER)/include -o build/mrhat-bq25622.dts.pre mrhat-bq25622.dts

build/mrhat-bq25622.dtbo: build/mrhat-bq25622.dts.pre
	mkdir -p build/
	dtc  -I dts -O dtb -o build/mrhat-bq25622.dtbo build/mrhat-bq25622.dts.pre

deploy: all
	rsync -avhz --progress build/mrhat-bq25622_$(VERSION)-1_armhf.deb $(TARGET):/tmp/
	ssh $(TARGET) -- sudo dpkg -r mrhat-bq25622
	ssh $(TARGET) -- sudo dpkg -i /tmp/mrhat-bq25622_$(VERSION)-1_armhf.deb

quickdeploy: build/bq2562x_charger.ko
	scp build/bq2562x_charger.ko $(TARGET):/tmp/
	ssh $(TARGET) -- sudo cp /tmp/bq2562x_charger.ko /lib/modules/$(KVER)/
	ssh $(TARGET) -- "sudo rmmod bq2562x_charger || true"
	ssh $(TARGET) -- "sudo modprobe bq2562x_charger"
	

.PHONY: clean all deploy quickdeploy

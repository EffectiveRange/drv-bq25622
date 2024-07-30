
VERSION = $(shell grep Version: mrhat-bq25622/DEBIAN/control | cut -d' ' -f2)
# TODO: build module for all kernel versions
KVER ?= 6.1.21+
TARGET ?=  $(error TARGET not specified for deploy )

all: build/mrhat-bq25622_$(VERSION)-1_armhf.deb
	@true

build/mrhat-bq25622_$(VERSION)-1_armhf.deb : driver build/mrhat-bq25622.dtbo mrhat-bq25622/DEBIAN/*
	mkdir -p build
	mkdir -p mrhat-bq25622/lib/modules/$(KVER)
	mkdir -p mrhat-bq25622/boot/overlays/
	cp build/mrhat-bq25622.dtbo mrhat-bq25622/boot/overlays/
	dpkg-deb --root-owner-group --build mrhat-bq25622 build/mrhat-bq25622_$(VERSION)-1_armhf.deb

mrhat-bq25622/lib/modules/$(KVER)/bq2562x_charger.ko: driver/*.c driver/*.h driver/Makefile
	mkdir -p build
	mkdir -p mrhat-bq25622/lib/modules/$(KVER)/
	rsync --delete -r  ./driver/ /tmp/drv-bq25622
	schroot -c buildroot -u root -d /tmp/drv-bq25622 -- make KVER=$(KVER) BQCFLAGS=$(BQCFLAGS)
	cp /tmp/drv-bq25622/bq2562x_charger.ko mrhat-bq25622/lib/modules/$(KVER)/bq2562x_charger.ko

driver: mrhat-bq25622/lib/modules/$(KVER)/bq2562x_charger.ko
	@true

clean:
	rm -rf mrhat-bq25622/boot/ mrhat-bq25622/lib/ build/

build/mrhat-bq25622.dts.pre: mrhat-bq25622.dts
	mkdir -p build/
	cpp -nostdinc -undef -x assembler-with-cpp -I/var/chroot/buildroot/usr/src/linux-headers-$(KVER)/include -o build/mrhat-bq25622.dts.pre mrhat-bq25622.dts

build/mrhat-bq25622.dtbo: build/mrhat-bq25622.dts.pre
	mkdir -p build/
	dtc  -I dts -O dtb -o build/mrhat-bq25622.dtbo build/mrhat-bq25622.dts.pre

deploy: all
	rsync -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" -avhz --progress build/mrhat-bq25622_$(VERSION)-1_armhf.deb $(TARGET):/tmp/
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo dpkg -r mrhat-bq25622
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo dpkg -i /tmp/mrhat-bq25622_$(VERSION)-1_armhf.deb
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo sed -ri '/^\s*dtoverlay=mrhat-bq25622/d' /boot/config.txt
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- "echo 'dtoverlay=mrhat-bq25622' | sudo tee -a /boot/config.txt"

quickdeploy: driver
	scp mrhat-bq25622/lib/modules/$(KVER)/bq2562x_charger.ko $(TARGET):/tmp/
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo cp /tmp/bq2562x_charger.ko /lib/modules/$(KVER)/
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- "sudo rmmod bq2562x_charger || true"
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- "sudo modprobe bq2562x_charger || true"
	

.PHONY: clean all deploy quickdeploy driver

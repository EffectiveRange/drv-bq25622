import argparse
import yaml
import jinja2

template = """\
VERSION = $(shell grep Version: staging/DEBIAN/control | cut -d' ' -f2)
# TODO: build module for all kernel versions
KVER ?= 6.1.21+
TARGET ?=  $(error TARGET not specified for deploy )
DISTRO = $(shell grep VERSION_CODENAME= /home/crossbuilder/target/target | cut -d'=' -f2)
ARCH = $(shell grep TARGET_ARCH= /home/crossbuilder/target/target | cut -d'=' -f2)
KBASEVER = $(shell echo "$(KVER)" | cut -d'-' -f1)

all: build/{{ project }}_$(VERSION)-1_$(ARCH).deb
	@true

build/{{ project }}_$(VERSION)-1_$(ARCH).deb : driver build/{{ project }}.dtbo staging/DEBIAN/*
	mkdir -p build
	mkdir -p staging//lib/modules/$(KVER)
	if [ "$(DISTRO)" = "bullseye" ]; then \\
		mkdir -p staging//boot/overlays/; \\
		cp build/{{ project }}.dtbo staging/boot/overlays/ ;\\
	else \\
		mkdir -p staging/boot/firmware/overlays/; \\
		cp build/{{ project }}.dtbo staging//boot/firmware/overlays/; \\
	fi
	dpkg-deb --root-owner-group --build staging build/{{ project }}_$(VERSION)-1_$(ARCH).deb

staging/lib/modules/$(KVER)/{{ modulename }}.ko: {{ sourcedir }}/*.c {{ sourcedir }}/*.h {{ sourcedir }}/Makefile
	mkdir -p build
	mkdir -p staging/lib/modules/$(KVER)/
	rsync --delete -r  ./{{ sourcedir }}/ /tmp/drv-{{ project }}
	schroot -c buildroot -u root -d /tmp/drv-{{ project }} -- make KVER=$(KVER) {{ kbuild_flags }}
	cp /tmp/drv-{{ project }}/{{ modulename }}.ko staging/lib/modules/$(KVER)/{{ modulename }}.ko

driver: staging/lib/modules/$(KVER)/{{ modulename }}.ko
	@true

clean:
	rm -rf staging/boot/ staging/lib/ build/

build/{{ project }}.dts.pre: {{ project }}.dts
	mkdir -p build/
	if [ "$(DISTRO)" = "bullseye" ]; then \\
		cpp -nostdinc -undef -x assembler-with-cpp -I/var/chroot/buildroot/usr/src/linux-headers-$(KVER)/include -o build/{{ project }}.dts.pre {{ project }}.dts ;\\
	else \
		KHDR_DIR=`ls -d1 /var/chroot/buildroot/usr/src/*$(KBASEVER)*-common-rpi`; \\
		cpp -nostdinc -undef -x assembler-with-cpp -I$${KHDR_DIR}/include -I/var/chroot/buildroot/usr/src/linux-headers-$(KVER)/include -o build/{{ project }}.dts.pre {{ project }}.dts ;\\
	fi
build/{{ project }}.dtbo: build/{{ project }}.dts.pre
	mkdir -p build/
	dtc  -I dts -O dtb -o build/{{ project }}.dtbo build/{{ project }}.dts.pre

deploy: all
	rsync -e "ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null" -avhz --progress build/{{ project }}_$(VERSION)-1_$(ARCH).deb $(TARGET):/tmp/
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo dpkg --force-all -i  /tmp/{{ project }}_$(VERSION)-1_$(ARCH).deb
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo sed -ri '/^\s*dtoverlay={{ project }}/d' /boot/config.txt
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- "echo 'dtoverlay={{ project }}:battery_diag=true' | sudo tee -a /boot/config.txt"

quickdeploy: driver
	scp {{ project }}/lib/modules/$(KVER)/{{ modulename }}.ko $(TARGET):/tmp/
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- "sudo rmmod {{ modulename }} || true"
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- sudo cp /tmp/{{ modulename }}.ko /lib/modules/$(KVER)/
	ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(TARGET) -- "sudo modprobe {{ modulename }} || true"
	

.PHONY: clean all deploy quickdeploy driver
"""


def get_args():
    parser = argparse.ArgumentParser(
        description="Cross Driver Configurator",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "projectdir",
        type=str,
        help="path to project roo directory containing a'drivercfg.yaml' file",
    )
    parser.add_argument("--build", action="store_true", help="build the driver")
    return parser.parse_args()


def main():
    args = get_args()

    with open(f"{args.projectdir}/drivercfg.yaml") as f:
        data = yaml.safe_load(f)
    # print(data)
    jtmpl = jinja2.Template(template)
    jtmpl.globals["project"] = data["project"]
    jtmpl.globals["modulename"] = data["modulename"]
    jtmpl.globals["sourcedir"] = data["sourcedir"]
    jtmpl.globals["kbuild_flags"] = data.get("kbuild_flags", "")
    print(jtmpl.render())


if __name__ == "__main__":
    main()

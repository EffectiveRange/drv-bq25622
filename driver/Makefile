obj-m += bq2562x_charger.o 

KVER ?= $(shell uname -r)
PWD := $(CURDIR) 
 
all: 
	make -C /lib/modules/$(KVER)/build M=$(PWD) modules 
 
clean: 
	make -C /lib/modules/$(KVER)/build M=$(PWD) clean
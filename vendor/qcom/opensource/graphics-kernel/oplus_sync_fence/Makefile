# SPDX-License-Identifier: GPL-2.0

all: modules

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 -C $(KERNEL_SRC) M=$(M) modules_install

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd Module.symvers
	rm -rf .tmp_versions

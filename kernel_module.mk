MMAP_ALLOC_KERNEL_DRIVER_SITE = package/mmap-alloc-kernel-driver
MMAP_ALLOC_KERNEL_DRIVER_SITE_METHOD = local

define MMAP_ALLOC_KERNEL_DRIVER_BUILD_CMDS
    $(TARGET_CC) $(@D)/mmap_alloc_test.c -o $(@D)/mmap_alloc_test
endef

define MMAP_ALLOC_KERNEL_DRIVER_INSTALL_TARGET_CMDS
    $(INSTALL) -D -m 0755 $(@D)/mmap_alloc_test $(TARGET_DIR)/usr/bin
endef

$(eval $(kernel-module))
$(eval $(generic-package))

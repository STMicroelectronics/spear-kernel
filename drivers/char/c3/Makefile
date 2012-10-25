#
# Makefile for the c3 driver.
#

ifeq ($(CONFIG_C3_DRIVER_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG -g
endif

ifeq ($(CONFIG_C3_DRIVER_SPEARBASIC_REVA),y)
EXTRA_CFLAGS += -DC3_SPEARBASIC_REVA_FFFF10xx
endif

ifeq ($(CONFIG_C3_DRIVER_SPEARBASIC_REVBA),y)
EXTRA_CFLAGS += -DC3_SPEARBASIC_REVBA_FFFF10xx
endif

ifeq ($(CONFIG_C3_DRIVER_SPEAR13xx),y)
EXTRA_CFLAGS += -DC3_SPEAR13xx
endif

ifeq ($(CONFIG_C3_CRYPTOAPI_INTEGRATION),m)
EXTRA_CFLAGS += -DCRYPTOAPI
endif

O_FILES_C3 :=\
 c3_autotest.o\
 c3_char_dev_driver.o\
 c3_char_dev_driver_instructions.o\
 c3_driver_core.o\
 c3_driver_interface.o\
 c3_irq.o\
 c3_mpcm.o\
 c3_registers.o\
 c3_spear.o\

LINK = $(LD) -r -o

c3_driver-y :=  $(O_FILES_C3)

obj-$(CONFIG_C3_DRIVER)+= c3_driver.o

obj-$(CONFIG_C3_CRYPTOAPI_INTEGRATION)+=c3_cryptoapi.o


#
# SPEAr Platform specific Makefile
#

# Common support
obj-y	:= clcd.o restart.o time.o

obj-$(CONFIG_ARCH_SPEAR3XX)	+= pl080.o shirq.o
obj-$(CONFIG_ARCH_SPEAR6XX)	+= pl080.o

ifeq ($(CONFIG_SUSPEND),y)
obj-$(CONFIG_ARCH_SPEAR3XX)     += pm.o sleep.o
obj-$(CONFIG_ARCH_SPEAR6XX)     += pm.o sleep.o
endif

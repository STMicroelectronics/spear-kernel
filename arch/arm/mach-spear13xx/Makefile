#
# Makefile for SPEAr13XX machine series
#

# common files
obj-y					+= spear13xx.o fsmc-nor.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o
obj-$(CONFIG_PCIEPORTBUS)		+= pcie.o
obj-$(CONFIG_PM)			+= pm.o sleep.o
obj-y					+= db9000_lcd.o

# spear1300 specific files
obj-$(CONFIG_CPU_SPEAR1300)		+= clock.o spear1300.o

# spear1300 boards files
obj-$(CONFIG_MACH_SPEAR1300_EVB)	+= spear1300_evb.o

# spear1310 specific files
obj-$(CONFIG_CPU_SPEAR1310)		+= clock.o spear1310.o

# spear1310 boards files
obj-$(CONFIG_MACH_SPEAR1310_EVB)	+= spear1310_evb.o

# spear1340 specific files
obj-$(CONFIG_CPU_SPEAR1340)		+= spear1340_clock.o spear1340.o

# spear1340 boards files
obj-$(CONFIG_MACH_SPEAR1340_EVB)	+= spear1340_evb.o

# spear900 specific files
obj-$(CONFIG_CPU_SPEAR900)		+= clock.o spear900.o
# spear900 boards files
obj-$(CONFIG_MACH_SPEAR900_EVB)	+= spear900_evb.o

#
# Makefile for SPEAr13XX machine series
#

# common files
obj-y					+= spear13xx.o clock.o
obj-$(CONFIG_SMP)			+= platsmp.o headsmp.o
obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o

# spear1300 specific files
obj-$(CONFIG_MACH_SPEAR1300)		+= spear1300.o

# spear1300 boards files
obj-$(CONFIG_BOARD_SPEAR1300_EVB)	+= spear1300_evb.o

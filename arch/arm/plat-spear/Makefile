#
# SPEAr Platform specific Makefile
#

# Common support
obj-y	:= clcd.o clock.o pll_clk.o smi.o time.o

obj-$(CONFIG_ARCH_SPEAR13XX)	+= padmux.o
obj-$(CONFIG_ARCH_SPEAR3XX)	+= shirq.o padmux.o pl080.o udc.o
obj-$(CONFIG_ARCH_SPEAR6XX)	+= pl080.o udc.o
obj-$(CONFIG_CPU_FREQ)		+= cpufreq.o
obj-$(CONFIG_CPU_SPEAR1300)	+= udc.o
obj-$(CONFIG_CPU_SPEAR1310_REVA) += plgpio.o udc.o
obj-$(CONFIG_CPU_SPEAR1310)	+= plgpio.o
obj-$(CONFIG_CPU_SPEAR1340)	+= plgpio.o
obj-$(CONFIG_CPU_SPEAR900)	+= udc.o
obj-$(CONFIG_CPU_SPEAR310)	+= plgpio.o
obj-$(CONFIG_CPU_SPEAR320)	+= plgpio.o
obj-$(CONFIG_SPEAR_HDLC)	+= e1phy.o
obj-$(CONFIG_SPEAR_PWM)		+= pwm.o

obj-$(CONFIG_MACH_SPEAR1300_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR1310_REVA_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR1310_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR1340_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR300_EVB)		+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR310_EVB)		+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR320_EVB)		+= i2c_eval_board.o
obj-$(CONFIG_MACH_SPEAR600_EVB)		+= i2c_eval_board.o
obj-$(CONFIG_SPEAR_PLUG_BOARDS)		+= plug_boards.o

ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_ARCH_SPEAR3XX)	+= pm.o sleep.o cpu_suspend.o
obj-$(CONFIG_ARCH_SPEAR6XX)	+= pm.o sleep.o cpu_suspend.o
obj-$(CONFIG_ARCH_SPEAR13XX)	+= cpu_suspend.o
endif

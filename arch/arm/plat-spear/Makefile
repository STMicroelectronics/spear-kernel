#
# SPEAr Platform specific Makefile
#

# Common support
obj-y	:= clcd.o clock.o pll_clk.o time.o

obj-$(CONFIG_ARCH_SPEAR3XX)	+= shirq.o padmux.o
obj-$(CONFIG_MACH_SPEAR310)	+= plgpio.o
obj-$(CONFIG_MACH_SPEAR320)	+= plgpio.o
obj-$(CONFIG_SPEAR_PWM)		+= pwm.o

obj-$(CONFIG_BOARD_SPEAR1310_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_BOARD_SPEAR1300_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_BOARD_SPEAR300_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_BOARD_SPEAR310_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_BOARD_SPEAR320_EVB)	+= i2c_eval_board.o
obj-$(CONFIG_BOARD_SPEAR600_EVB)	+= i2c_eval_board.o

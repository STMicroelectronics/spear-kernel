#ifndef __AD9889B_H
#define __AD9889B_H

#include <linux/fb.h>

struct ad9889b_pdata {
	u32 irq_gpio;
	u32 irq_type;
	int fb;
};
#endif /* __AD9889B_H */


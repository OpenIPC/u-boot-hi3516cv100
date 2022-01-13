#include <common.h>
#include <command.h>
#include <asm/arch/gpio.h>

int do_gpio(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int i;
	for (i = 1; i < argc; ++i) {
		char *p = argv[i];
		while (*p) {
			int gpio = simple_strtoul (p,(char **)&p,10);
			int val = 1;
			if (*p == '!') {
				val = 0;
				++p;
			}
			printf ("Set GPIO: %d->%d\n",gpio,val);
			gpio_set_mode (gpio,GPIO_MODE_OUTPUT);
			gpio_set (gpio,val);
			if (*p++ != ',')
				break;
		}
	}
	return 0;
}

U_BOOT_CMD(
	gpio,	CONFIG_SYS_MAXARGS,	1,	do_gpio,
	"Set GPIO states",
	"[args..]\n"
	""
);

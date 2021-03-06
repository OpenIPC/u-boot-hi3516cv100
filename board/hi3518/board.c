/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
 *
 * (C) Copyright 2003
 * Texas Instruments, <www.ti.com>
 * Kshitij Gupta <Kshitij@ti.com>
 *
 * (C) Copyright 2004
 * ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/arch/platform.h>

static int boot_media = BOOT_MEDIA_UNKNOW;

#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

#define COMP_MODE_ENABLE ((unsigned int)0x0000EAEF)

static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n"
		"subs %0, %1, #1\n"
		"bne 1b" : "=r" (loops) : "0" (loops));
}

/* get uboot start media. */
int get_boot_media(void)
{
	return boot_media;
}

void boot_flag_init(void)
{
	unsigned long ret;

	/* get boot flag */
	ret = __raw_readl(REG_BASE_SCTL + REG_SYSSTAT);
	ret >>= 5;
	ret = (ret & 0x1);

	switch (ret) {
	case 0x0:
		boot_media = BOOT_MEDIA_SPIFLASH;
		break;
	case 0x1:
		boot_media = BOOT_MEDIA_NAND;
		break;
	default:
		boot_media = BOOT_MEDIA_UNKNOW;
		break;
	}
}

/*
 * Miscellaneous platform dependent initialisations
 */
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_arch_number = MACH_TYPE_HI3518;
	gd->bd->bi_boot_params = CFG_BOOT_PARAMS;
	gd->flags = 0;

	boot_flag_init();

	return 0;
}

void detect_memory(void) {
  ulong tested_ram =
      get_ram_size((long *)CFG_DDR_PHYS_OFFSET, CFG_DDR_SIZE) / 1024 / 1024;
  printf("RAM size: %dMB\n", tested_ram);

  char msize[128];
  sprintf(msize, "%dM", tested_ram);
  setenv("totalmem", msize);
}

void do_phy_init(void) {
  char *mdio_intf = NULL;

  mdio_intf = getenv("mdio_intf");
  if (mdio_intf) {
	printf("PHY Init... ");
    if (!strncmp(mdio_intf, "mii", 3)) {
      writel(0x1, 0x200f005c); // 25 MHz RMII
      writel(0x0, 0x200f0070); // GPIO 1_3 PHYRST on XM boards
      writel(0x2, 0x200300cc); // MII mode
	  printf("mii\n");
    } else if (!strncmp(mdio_intf, "rmii", 4)) {
      writel(0x3, 0x200f005c); // 50 MHz RMII
      writel(0x0, 0x200f0070); // GPIO 1_3 PHYRST on XM boards
      writel(0xa, 0x200300cc); // RMII mode
	  printf("rmii\n");
    }
  }
}

int misc_init_r(void)
{
    detect_memory();
#ifdef CONFIG_RANDOM_ETHADDR
	random_init_r();
#endif
	setenv("verify", "n");
	do_phy_init();
#ifdef CONFIG_AUTO_UPDATE
	extern int do_auto_update(void);
#ifdef CFG_MMU_HANDLEOK
	dcache_stop();
#endif
	do_auto_update();
#ifdef CFG_MMU_HANDLEOK
	dcache_start();
#endif
#endif /* CONFIG_AUTO_UPDATE */
	return 0;
}

int dram_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_dram[0].start = CFG_DDR_PHYS_OFFSET;
	gd->bd->bi_dram[0].size = CFG_DDR_SIZE;

	return 0;
}

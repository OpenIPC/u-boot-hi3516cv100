#
# (C) Copyright 2004-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	= $(obj)libcommon.a

COBJS-$(CONFIG_BOOTROM_SUPPORT) += download_process.o

# core
COBJS-y += main.o
COBJS-y += console.o
COBJS-y += command.o
COBJS-y += dlmalloc.o
COBJS-y += exports.o
COBJS-$(CONFIG_SYS_HUSH_PARSER) += hush.o
COBJS-y += image.o
COBJS-y += memsize.o
COBJS-y += s_record.o
COBJS-$(CONFIG_SERIAL_MULTI) += serial.o
COBJS-y += stdio.o
COBJS-y += xyzModem.o
COBJS-y += flash_part.o
COBJS-$(CONFIG_CMD_SF) += spiflash_logif.o
COBJS-$(CONFIG_CMD_NAND) += nand_logif.o

# core command
ifndef CONFIG_HI3536_A7
COBJS-y += cmd_getinfo.o
endif
COBJS-y += cmd_boot.o
COBJS-y += cmd_bootm.o
COBJS-y += cmd_help.o
COBJS-y += cmd_nvedit.o
COBJS-y += cmd_version.o
#ddr training command
COBJS-$(CONFIG_DDR_TRAINING) += cmd_ddrtraining.o
COBJS-$(CONFIG_DDR_TRAINING_V200) += cmd_ddrtrainingv200.o

# environment
COBJS-y += env_common.o env_common_func.o
COBJS-$(CONFIG_ENV_IS_IN_DATAFLASH) += env_dataflash.o
COBJS-$(CONFIG_ENV_IS_IN_EEPROM) += env_eeprom.o
COBJS-$(CONFIG_ENV_IS_EMBEDDED) += env_embedded.o
COBJS-$(CONFIG_ENV_IS_IN_EEPROM) += env_embedded.o
COBJS-$(CONFIG_ENV_IS_IN_FLASH) += env_embedded.o
COBJS-$(CONFIG_ENV_IS_IN_NVRAM) += env_embedded.o
COBJS-$(CONFIG_ENV_IS_IN_FLASH) += env_flash.o
COBJS-$(CONFIG_ENV_IS_IN_MG_DISK) += env_mgdisk.o
COBJS-$(CONFIG_ENV_IS_IN_NAND) += env_nand.o
COBJS-$(CONFIG_ENV_IS_IN_NVRAM) += env_nvram.o
COBJS-$(CONFIG_ENV_IS_IN_ONENAND) += env_onenand.o
COBJS-$(CONFIG_ENV_IS_IN_SPI_FLASH) += env_sf.o
COBJS-$(CONFIG_ENV_IS_IN_EMMC) += env_emmc.o
COBJS-$(CONFIG_ENV_IS_NOWHERE) += env_nowhere.o

# command
COBJS-$(CONFIG_CMD_AMBAPP) += cmd_ambapp.o
COBJS-$(CONFIG_SOURCE) += cmd_source.o
COBJS-$(CONFIG_CMD_SOURCE) += cmd_source.o
COBJS-$(CONFIG_CMD_BDI) += cmd_bdinfo.o
COBJS-$(CONFIG_CMD_BEDBUG) += bedbug.o cmd_bedbug.o
COBJS-$(CONFIG_CMD_BMP) += cmd_bmp.o
COBJS-$(CONFIG_CMD_BOOTLDR) += cmd_bootldr.o
COBJS-$(CONFIG_CMD_CACHE) += cmd_cache.o
COBJS-$(CONFIG_CMD_CONSOLE) += cmd_console.o
COBJS-$(CONFIG_CMD_CPLBINFO) += cmd_cplbinfo.o
COBJS-$(CONFIG_DATAFLASH_MMC_SELECT) += cmd_dataflash_mmc_mux.o
COBJS-$(CONFIG_CMD_DATE) += cmd_date.o
ifdef CONFIG_4xx
COBJS-$(CONFIG_CMD_SETGETDCR) += cmd_dcr.o
endif
ifdef CONFIG_POST
COBJS-$(CONFIG_CMD_DIAG) += cmd_diag.o
endif
COBJS-$(CONFIG_CMD_DISPLAY) += cmd_display.o
COBJS-$(CONFIG_CMD_DTT) += cmd_dtt.o
COBJS-$(CONFIG_CMD_ECHO) += cmd_echo.o
COBJS-$(CONFIG_ENV_IS_IN_EEPROM) += cmd_eeprom.o
COBJS-$(CONFIG_CMD_EEPROM) += cmd_eeprom.o
COBJS-$(CONFIG_CMD_ELF) += cmd_elf.o
COBJS-$(CONFIG_SYS_HUSH_PARSER) += cmd_exit.o
ifndef CONFIG_HI3536_A7
COBJS-$(CONFIG_CMD_EXT2) += cmd_ext2.o
COBJS-$(CONFIG_CMD_FAT) += cmd_fat.o
endif
COBJS-$(CONFIG_CMD_FDC)$(CONFIG_CMD_FDOS) += cmd_fdc.o
COBJS-$(CONFIG_OF_LIBFDT) += cmd_fdt.o fdt_support.o
COBJS-$(CONFIG_CMD_FDOS) += cmd_fdos.o
COBJS-$(CONFIG_CMD_FLASH) += cmd_flash.o
ifdef CONFIG_FPGA
COBJS-$(CONFIG_CMD_FPGA) += cmd_fpga.o
endif
COBJS-$(CONFIG_CMD_I2C) += cmd_i2c.o
COBJS-$(CONFIG_CMD_IDE) += cmd_ide.o
COBJS-$(CONFIG_CMD_IMMAP) += cmd_immap.o
COBJS-$(CONFIG_CMD_IRQ) += cmd_irq.o
COBJS-$(CONFIG_CMD_ITEST) += cmd_itest.o
COBJS-$(CONFIG_CMD_JFFS2) += cmd_jffs2.o
COBJS-$(CONFIG_CMD_CRAMFS) += cmd_cramfs.o
COBJS-$(CONFIG_CMD_LICENSE) += cmd_license.o
COBJS-y += cmd_load.o
COBJS-$(CONFIG_LOGBUFFER) += cmd_log.o
COBJS-$(CONFIG_ID_EEPROM) += cmd_mac.o
COBJS-$(CONFIG_CMD_MEMORY) += cmd_mem.o
COBJS-$(CONFIG_CMD_MFSL) += cmd_mfsl.o
COBJS-$(CONFIG_CMD_MG_DISK) += cmd_mgdisk.o
COBJS-$(CONFIG_MII) += miiphyutil.o
COBJS-$(CONFIG_CMD_MII) += miiphyutil.o
ifndef CONFIG_HI3536_A7
COBJS-$(CONFIG_CMD_MII) += cmd_mii.o
endif
COBJS-$(CONFIG_CMD_MISC) += cmd_misc.o
ifndef CONFIG_HI3536_A7
COBJS-$(CONFIG_CMD_MMC) += cmd_mmc.o
endif
COBJS-$(CONFIG_MP) += cmd_mp.o
COBJS-$(CONFIG_CMD_MTDPARTS) += cmd_mtdparts.o
COBJS-$(CONFIG_CMD_NAND) += cmd_nand.o
ifndef CONFIG_HI3536_A7
COBJS-$(CONFIG_CMD_NET) += cmd_net.o
endif
COBJS-$(CONFIG_CMD_ONENAND) += cmd_onenand.o
COBJS-$(CONFIG_CMD_OTP) += cmd_otp.o
ifdef CONFIG_PCI
COBJS-$(CONFIG_CMD_PCI) += cmd_pci.o
endif
COBJS-y += cmd_pcmcia.o
COBJS-$(CONFIG_CMD_PORTIO) += cmd_portio.o
COBJS-$(CONFIG_CMD_REGINFO) += cmd_reginfo.o
COBJS-$(CONFIG_CMD_REISER) += cmd_reiser.o
COBJS-$(CONFIG_CMD_SATA) += cmd_sata.o
COBJS-$(CONFIG_CMD_SF) += cmd_sf.o
COBJS-$(CONFIG_CMD_SCSI) += cmd_scsi.o
COBJS-$(CONFIG_CMD_SETEXPR) += cmd_setexpr.o
COBJS-$(CONFIG_CMD_SPI) += cmd_spi.o
COBJS-$(CONFIG_CMD_SPIBOOTLDR) += cmd_spibootldr.o
COBJS-$(CONFIG_CMD_STRINGS) += cmd_strings.o
COBJS-$(CONFIG_CMD_TERMINAL) += cmd_terminal.o
COBJS-$(CONFIG_SYS_HUSH_PARSER) += cmd_test.o
COBJS-$(CONFIG_CMD_TSI148) += cmd_tsi148.o
COBJS-$(CONFIG_CMD_UBI) += cmd_ubi.o
COBJS-$(CONFIG_CMD_UBIFS) += cmd_ubifs.o
COBJS-$(CONFIG_CMD_UNIVERSE) += cmd_universe.o
COBJS-$(CONFIG_HAS_SLAVE) += cmd_slavestart.o
ifdef CONFIG_CMD_USB
COBJS-y += cmd_usb.o
COBJS-y += usb.o
COBJS-$(CONFIG_USB_STORAGE) += usb_storage.o
endif
COBJS-$(CONFIG_CMD_XIMG) += cmd_ximg.o
COBJS-$(CONFIG_YAFFS2) += cmd_yaffs2.o
COBJS-$(CONFIG_VFD) += cmd_vfd.o

# others
COBJS-$(CONFIG_DDR_SPD) += ddr_spd.o
COBJS-$(CONFIG_HWCONFIG) += hwconfig.o
COBJS-$(CONFIG_CONSOLE_MUX) += iomux.o
COBJS-y += flash.o
COBJS-$(CONFIG_CMD_KGDB) += kgdb.o kgdb_stubs.o
COBJS-$(CONFIG_KALLSYMS) += kallsyms.o
COBJS-$(CONFIG_LCD) += lcd.o
COBJS-$(CONFIG_LYNXKDI) += lynxkdi.o
COBJS-$(CONFIG_MODEM_SUPPORT) += modem.o
COBJS-$(CONFIG_UPDATE_TFTP) += update.o
COBJS-$(CONFIG_USB_KEYBOARD) += usb_kbd.o
AOBJS-$(CONFIG_DDR_TRAINING_V300) += ddr_training_entry.o
COBJS-$(CONFIG_DDR_TRAINING_V300) += cmd_ddr_training_v300.o
COBJS-$(CONFIG_DDR_TRAINING_HI3535)  += hi3535_ddr_training_printf.o
COBJS-$(CONFIG_DDR_TRAINING_HI3516A)  += hi3516a_ddr_training_printf.o
COBJS-$(CONFIG_DDR_TRAINING_V2) += cmd_ddr_training_v2.o
COBJS-$(CONFIG_CMD_GPIO)  += cmd_gpio.o

ifeq ($(CONFIG_AUDIO_ENABLE), y)
ifeq ($(CONFIG_PRODUCTNAME), "hi3516a")
COBJS-y += cmd_ao.o
endif
endif
ifndef CONFIG_HI3536_A7
sinclude Makefile-osd
endif

AOBJS	:= $(sort $(AOBJS-y))
COBJS	:= $(sort $(COBJS-y))
SRCS	:= $(AOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(AOBJS) $(COBJS))

SRCS    += $(obj)crc32.c

CPPFLAGS += -I..

all:	$(LIB) $(AOBJS) bootss2

$(LIB): $(obj).depend $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

bootss2:
ifdef CONFIG_SNAPSHOT_BOOT
	rm bootss2.a -rf
ifeq ($(CROSS_COMPILE),arm-hisiv300-linux-)
ifdef CONFIG_HI3518EV200
	cp cmd_bootss2_v300_hi3518ev200 bootss2.a -rf
else
	cp cmd_bootss2_v300 bootss2.a -rf
endif
endif

ifeq ($(CROSS_COMPILE),arm-hisiv400-linux-)
ifdef CONFIG_HI3518EV200
	cp cmd_bootss2_v400_hi3518ev200 bootss2.a -rf
else
	cp cmd_bootss2_v400 bootss2.a -rf
endif
endif
endif
$(obj)env_embedded.o: $(src)env_embedded.c $(obj)../tools/envcrc
	$(CC) $(AFLAGS) -Wa,--no-warn \
		-DENV_CRC=$(shell $(obj)../tools/envcrc) \
		-c -o $@ $(src)env_embedded.c

$(obj)../tools/envcrc:
	$(MAKE) -C ../tools

$(obj)crc32.c:
	ln -s $(src)$(SRCTREE)/lib/crc32.c $(obj)crc32.c

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

# u-boot-hi3516cv100
U-Boot for hi3516cv100 group SoC's

## reg_info tables

`u-boot.bin` is not flashable on its own. `start.S` reserves 4 KiB at image
offset 0x40 and, running from flash at reset, walks it as a table of
`(addr, value, delay, attr)` records: the PLLs, the DDR controller and PHY,
pin mux. The build leaves that region zero; an image flashed without a table
never reaches DDR, prints nothing, and needs an SPI programmer to recover
(OpenIPC/firmware#2593). CI writes `reg_info_<soc>.bin` in at 0x40, refuses
to publish an image whose table is empty, and publishes only the SoCs listed
here:

Every table is the reference-board `reg_info_<chip>.bin` from HiSilicon's
Hi3518_SDK_V1.0.B.0 (`osdrv/tools/pc_tools/uboot_tools/`, generated from the
`uboot-Hi35xx-bvt_No2_*.xlsm` workbooks beside it), zero-padded to 4 KiB as the
SDK's `mkboot.sh` does.

| SoC | Table | SDK source | Also checked against |
|---|---|---|---|
| hi3516cv100 | `reg_info_hi3516cv100.bin` | `reg_info_hi3516c.bin` (440/220/440 MHz) | 0x40..0x1040 of the OpenIPC hi3516cv100 U-Boot built 2022-11-14, read from mtd0 of a Hi3516CV100 + IMX122 board (128 MB DDR) that boots with it: byte-identical |
| hi3518av100 | `reg_info_hi3518av100.bin` | `reg_info_hi3518a.bin` (440/200/400 MHz) | nothing: OpenIPC had not published a table for it before |
| hi3518cv100 | `reg_info_hi3518cv100.bin` | `reg_info_hi3518c.bin` (440/200/400 MHz) | OpenIPC/distributor `reg_info_hi3518cv100.reg` (a19c1f7): byte-identical |
| hi3518ev100 | `reg_info_hi3518ev100.bin` | `reg_info_hi3518e.bin` (440/200/400 MHz, low power) | OpenIPC/distributor `reg_info_hi3518ev100.reg` (a19c1f7): byte-identical |

Older SDK copies of these tables (e.g. the 2014 ones in Hi-Spy/hi_u-boot) write
some DDR PHY registers at 0x2009xxxx/0x2011xxxx; V1.0.B.0 has them at 0x2012xxxx.

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

| SoC | Table | Source |
|---|---|---|
| hi3516cv100 | `reg_info_hi3516cv100.bin` | 0x40..0x1040 of the OpenIPC hi3516cv100 U-Boot built 2022-11-14, read from mtd0 of a Hi3516CV100 + IMX122 board (128 MB DDR) that runs it. That binary matches this tree's build byte for byte apart from the build-date string and this table. |
| hi3518cv100 | `reg_info_hi3518cv100.bin` | `reg_info_hi3518cv100.reg` from OpenIPC/distributor (added 2023-11-17, a19c1f7), OpenIPC's U-Boot build for this family before this repo's CI. It is HiSilicon's SDK `reg_info_hi3518c.bin` with the PHY writes moved from 0x2009xxxx (UART1) to the DDR PHY at 0x2012xxxx. |
| hi3518ev100 | `reg_info_hi3518ev100.bin` | `reg_info_hi3518ev100.reg` from OpenIPC/distributor, same history. It is the SDK `reg_info_hi3518e.bin` with two DDR PHY drive values changed and two PHY writes added. |
| hi3518av100 | none yet | built, not published; OpenIPC has never published a table for it |

distributor's `reg_info_hi3516cv100.reg` is byte-identical to `reg_info_hi3516cv100.bin`.

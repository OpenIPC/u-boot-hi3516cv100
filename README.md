# u-boot-hi3516cv100

OpenIPC U-Boot for the HiSilicon V1 family:

- Hi3516CV100
- Hi3518AV100
- Hi3518CV100
- Hi3518EV100

The GitHub Actions workflow builds the supported configurations with the
HiSilicon V100 SDK toolchain, checks that every image fits the 256 KiB boot
partition, smoke-tests the artifacts with qemu-hisilicon, and publishes them
to the OpenIPC firmware release.

## Published artifacts

| SoC / memory variant | U-Boot config | Artifact |
|---|---|---|
| Hi3516CV100 | `hi3516c_config` | `u-boot-hi3516cv100-universal.bin` |
| Hi3518AV100 | `hi3518a_config` | `u-boot-hi3518av100-universal.bin` |
| Hi3518CV100 | `hi3518c_config` | `u-boot-hi3518cv100-universal.bin` |
| Hi3518EV100 | `hi3518e_config` | `u-boot-hi3518ev100-universal.bin` |
| Hi3518EV100 DDR3 / 256 MiB | `hi3518e_ddr3_256m_config` | `u-boot-hi3518ev100-ddr3-256m-universal.bin` |

## Hi3518EV100 DDR3 / 256 MiB

The normal `hi3518e_config` remains unchanged.

The DDR3 / 256 MiB variant has two additional requirements:

1. `hi3518e_ddr3_256m_config` defines
   `CONFIG_HI3518EV100_DDR3_256M` and otherwise includes the normal
   `hi3518e.h` configuration. This raises only `CFG_DDR_SIZE` from
   128 MiB to 256 MiB, allowing `detect_memory()` to probe the complete
   physical RAM range.

2. After `u-boot.bin` is built, the workflow overlays the 4096-byte
   `reg_info_hi3518ev100_ddr3_256m.reg` cold-init table at offset `0x40`.
   CI reads the same 4096 bytes back and compares them byte-for-byte with
   the source table before publishing the artifact.

The register table was recovered from a vendor bootloader and the resulting
DDR initialization was validated on HiWatch DS-I203 hardware.

This U-Boot variant deliberately does not contain device policy. The normal
OpenIPC environment defaults remain unchanged, including `osmem=32M` and the
generic flash partition layout. Device-specific Linux memory allocation,
flash layout, Ethernet PHY settings and image sensor selection belong to the
firmware/device profile.

DDR initialization data is hardware-specific. Do not assume that
`u-boot-hi3518ev100-ddr3-256m-universal.bin` is suitable for every
Hi3518EV100 board simply because it has 256 MiB of DDR3.

`universal` is the existing OpenIPC release naming convention; it does not
mean that DDR initialization is universal between different boards.

## Build matrix

The workflow separates three values:

- `name` — published artifact name;
- `config` — U-Boot configuration target;
- `reg` — optional cold-init register table.

The four generic builds have an empty `reg` value and their `u-boot.bin` is
copied unchanged. The DDR3 / 256 MiB build selects its own configuration and
register table while using the same build and publication path.

## Manual build

Generic Hi3518EV100:

    make CROSS_COMPILE=arm-hisiv100-linux-uclibcgnueabi- hi3518e_config
    make CROSS_COMPILE=arm-hisiv100-linux-uclibcgnueabi- SUBDIRS= u-boot.bin

DDR3 / 256 MiB:

    make CROSS_COMPILE=arm-hisiv100-linux-uclibcgnueabi- hi3518e_ddr3_256m_config
    make CROSS_COMPILE=arm-hisiv100-linux-uclibcgnueabi- SUBDIRS= u-boot.bin
    cp u-boot.bin u-boot-hi3518ev100-ddr3-256m-universal.bin
    dd if=reg_info_hi3518ev100_ddr3_256m.reg of=u-boot-hi3518ev100-ddr3-256m-universal.bin bs=1 seek=64 conv=notrunc

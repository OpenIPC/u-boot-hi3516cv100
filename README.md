# Retired: the hi3516cv100 family is built in OpenIPC/u-boot

This repository is archived. U-Boot for Hi3516CV100, Hi3518AV100, Hi3518CV100
and Hi3518EV100 is now built from the same tree as every other HiSilicon and
Goke SoC OpenIPC supports: the `openipc-hisilicon` branch of
[OpenIPC/u-boot](https://github.com/OpenIPC/u-boot/tree/openipc-hisilicon),
with its CI on the `openipc` branch. Those builds are published as
`u-boot-hi3516cv100-nor.bin`, `u-boot-hi3518av100-nor.bin`,
`u-boot-hi3518cv100-nor.bin` and `u-boot-hi3518ev100-nor.bin` in the
[OpenIPC/firmware `latest`](https://github.com/OpenIPC/firmware/releases/tag/latest)
release; they find the flash layout at boot, fixed slots or the split NOR
image (OpenIPC/u-boot#77).

The U-Boot 2010.06 images this repository built, `u-boot-hi3516cv100-universal.bin`,
`u-boot-hi3518av100-universal.bin`, `u-boot-hi3518cv100-universal.bin` and
`u-boot-hi3518ev100-universal.bin`, stay on `latest` as they are: qemu-bench
CI boots the Hi3516CV100 one, defib still loads the Hi3518AV100 one, and
cameras in the field run them. They are no longer rebuilt.

The upstream U-Boot documentation is in `README`; the history here stays
readable for reference.

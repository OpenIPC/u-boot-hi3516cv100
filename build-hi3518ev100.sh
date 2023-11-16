#!/bin/sh

export CROSS_COMPILE=/usr/bin/arm-linux-gnueabi-

make distclean
make hi3518e_config
make -j4

# hisilicon-specific initialization header patch
dd if=u-boot.bin of=tmp1 bs=1 count=64
dd if=reg_info_hi3518e.reg of=tmp2 bs=4096 conv=sync
dd if=u-boot.bin of=tmp3 bs=1 skip=4160
cat tmp1 tmp2 tmp3 > u-boot.bin
rm -f tmp1 tmp2 tmp3

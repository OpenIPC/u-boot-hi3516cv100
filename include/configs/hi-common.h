#define CONFIG_BOOTARGS "mem=${osmem} console=ttyAMA0,115200 panic=20 root=/dev/mtdblock3 rootfstype=squashfs init=/init mtdparts=sfc:256k(boot),64k(env),2048k(kernel),5120k(rootfs),-(rootfs_data)"
#define CONFIG_BOOTCOMMAND "setenv setargs setenv bootargs ${bootargs}; run setargs; sf probe 0; sf read 0x82000000 0x50000 0x200000; bootm 0x82000000"

#define CONFIG_EXTRA_ENV_SETTINGS \
    "uk=mw.b 0x82000000 ff 1000000; tftpboot 0x82000000 uImage.${soc} && sf probe 0; sf erase 0x50000 0x200000; sf write 0x82000000 0x50000 ${filesize}\0" \
    "ur=mw.b 0x82000000 ff 1000000; tftpboot 0x82000000 rootfs.squashfs.${soc} && sf probe 0; sf erase 0x250000 0x500000; sf write 0x82000000 0x250000 ${filesize}\0" \
    "osmem=32M\0" \
    "soc="CONFIG_PRODUCTNAME"v100"

#undef CONFIG_OSD_ENABLE

#ifndef CONFIG_XM_COMPATIBLE
    #define CONFIG_ENV_OFFSET       0x40000
#else
    #define CONFIG_ENV_OFFSET       0x30000
#endif
#define CONFIG_ENV_SIZE         0x10000

#undef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT	"OpenIPC # "

#if (CONFIG_AUTO_UPDATE == 1)
#define CONFIG_AUTO_UPDATE_ADAPTATION   1
#define CONFIG_AUTO_SD_UPDATE     1

#ifndef CONFIG_MINI_BOOT
#define CONFIG_CMD_FAT          1
#endif
#endif

#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2 " > "

#define CONFIG_CMD_GPIO 1
#define CONFIG_CMD_RUN 1
#define CONFIG_SD_BOOT 1
#define CONFIG_SPI_BOOT 1
#define CONFIG_BOOTDELAY 1
#define CONFIG_KERNEL_LOAD_ADDR 0x82000000
#define CONFIG_CMD_TFTPPUT 1
#define CONFIG_CMD_FAT 1
#define CONFIG_CMD_FS_GENERIC 1
#define CONFIG_SPI_BLOCK_PROTECT 1
#define CONFIG_NETMASK 255.255.255.0
#define CONFIG_GATEWAYIP 192.168.1.1
#define CONFIG_SERVERIP 192.168.1.1
#define CONFIG_IPADDR 192.168.1.10

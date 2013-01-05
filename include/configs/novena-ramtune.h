/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6Q Sabre Lite board.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.		See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define NOVENA_RAMTUNE

#ifdef NOVENA_RAMTUNE
#define DEBUG
#undef CONFIG_CMD_SF
#undef CONFIG_CMD_IMXOTP
#undef CONFIG_FS
#undef CONFIG_CMD_SATA
#undef CONFIG_NET
#undef CONFIG_USB
#undef CONFIG_VIDEO

#undef CONFIG_I2C

#undef CONFIG_CMD_BMODE

#define CONFIG_CMD_BDI		/* bdinfo			*/
#define CONFIG_CMD_CONSOLE	/* coninfo			*/
#define CONFIG_CMD_ECHO		/* echo arguments		*/
#define CONFIG_CMD_EDITENV	/* editenv			*/
#define CONFIG_CMD_ITEST	/* Integer (and string) test	*/
#define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
//#define CONFIG_CMD_RUN		/* run command in env variable	*/

#undef CONFIG_BOOTM_LINUX
#undef CONFIG_BOOTM_NETBSD 
#undef CONFIG_BOOTM_RTEMS 

#undef CONFIG_GZIP 
#undef CONFIG_ZLIB 
#undef CONFIG_PARTITIONS 

#define CONFIG_BOOTCOMMAND \
       "echo Do nothing...; "

#undef CONFIG_SYS_HUSH_PARSER  // doesn't seem to work

#else
#define CONFIG_CMD_SATA
#define CONFIG_CMD_SF
#define CONFIG_CMD_IMXOTP
#define CONFIG_FS
#define CONFIG_CMD_SATA
#define CONFIG_NET
#define CONFIG_USB
#define CONFIG_VIDEO

/* Command definition */
#include <config_cmd_default.h>

/* Miscellaneous commands */
#define CONFIG_CMD_BMODE

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_SYS_HUSH_PARSER
#endif

#define CONFIG_MX6Q
#define CONFIG_SYS_MX6_HCLK	       24000000
#define CONFIG_SYS_MX6_CLK32	       32768
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_MACH_TYPE	0xffffffff

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN         (30 * 1024) // just 30k for mallocs

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	       UART2_BASE

#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   (0|(IMX_GPIO_NR(3, 19)<<8))
#define CONFIG_SF_DEFAULT_SPEED 25000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

/* I2C Configs */
#ifdef CONFIG_I2C
#define CONFIG_CMD_I2C
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#else
#define CONFIG_SYS_I2C_SPEED		100000
#endif

/* OCOTP Configs */
#ifdef CONFIG_CMD_IMXOTP
#define CONFIG_IMX_OTP
#define IMX_OTP_BASE			OCOTP_BASE_ADDR
#define IMX_OTP_ADDR_MAX		0x7F
#define IMX_OTP_DATA_ERROR_VAL		0xBADABADA
//#define IMX_OTPWRITE_ENABLED
#endif

/* MMC Configs */
#ifdef CONFIG_FS
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       2

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#endif

/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#ifdef CONFIG_NET
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_FEC_MXC_PHYMASK		(0xf << 4)	/* scan phy 4,5,6,7 */
#endif

#ifndef IPU_CPMEM_REG_BASE
#define IPU_CPMEM_REG_BASE     0x01000000
#define IPU_LUT_REG_BASE       0x01020000
#define IPU_SRM_REG_BASE       0x01040000
#define IPU_TPM_REG_BASE       0x01060000
#define IPU_DC_TMPL_REG_BASE   0x01080000
#define IPU_ISP_TBPR_REG_BASE  0x010C0000
#endif


#ifdef CONFIG_NET
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9021
#endif

/* USB Configs */
#ifdef CONFIG_USB
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_MXC_USB_PORT	1
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#endif

/* Framebuffer and LCD */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_IPUV3_CLK 260000000
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	       1

#define CONFIG_PREBOOT                 ""

#define CONFIG_LOADADDR			       0x10800000

#ifdef NOVENA_RAMTUNE
//#define CONFIG_SYS_TEXT_BASE           0x00920000 // at top
#define CONFIG_SYS_TEXT_BASE           0x009073e0   // at bottom
#else
#define CONFIG_SYS_TEXT_BASE	       0x17800000
#endif

/*
 * The boot ROM code copies U-Boot already into the RAM at
 * address CONFIG_SYS_TEXT_BASE.  To improve the boot time,
 * skip the relocation (i.e. the copy RAM -> RAM).
 *
 * ATTENTION: Please read
 *
 * http://lists.denx.de/pipermail/u-boot/2012-February/117140.html
 *
 * why you normally don't want this!
 */
#define CONFIG_SYS_SKIP_ARM_RELOCATION

#ifndef NOVENA_RAMTUNE
#define CONFIG_EXTRA_ENV_SETTINGS \
       "script=boot.scr\0" \
       "uimage=uimage-novena\0" \
	"console=ttymxc1\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
       "mmcdev=0\0" \
       "mmcpart=1\0" \
       "mmcroot=/dev/mmcblk0p2 rootwait rw\0" \
       "mmcargs=setenv bootargs console=${console},${baudrate} " \
	       "root=${mmcroot}\0" \
       "loadbootscript=" \
	       "fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
       "bootscript=echo Running bootscript from mmc ...; " \
	       "source\0" \
       "loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
       "mmcboot=echo Booting from mmc ...; " \
	       "run mmcargs; " \
	       "bootm\0" \
       "netargs=setenv bootargs console=${console},${baudrate} " \
	       "root=/dev/nfs " \
	       "ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
       "netboot=echo Booting from net ...; " \
	       "run netargs; " \
	       "dhcp ${uimage}; bootm\0" \

#define CONFIG_BOOTCOMMAND \
       "mmc dev ${mmcdev};" \
       "if mmc rescan ${mmcdev}; then " \
	       "if run loadbootscript; then " \
		       "run bootscript; " \
	       "else " \
		       "if run loaduimage; then " \
			       "run mmcboot; " \
		       "else run netboot; " \
		       "fi; " \
	       "fi; " \
       "else run netboot; fi"

#define CONFIG_ARP_TIMEOUT     200UL
#endif

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT	       "Novena U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE	       256

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x10010000

#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR
#define CONFIG_SYS_HZ		       1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#ifdef NOVENA_RAMTUNE
#define PHYS_SDRAM		       0x00907000
#define PHYS_SDRAM_SIZE	       (196 * 1024) // just 196k available in small boot environment
#else
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			       (2 * 1024 * 1024 * 1024)
#endif

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
       (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
       (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#ifndef NOVENA_RAMTUNE
#define CONFIG_ENV_IS_IN_MMC
/* #define CONFIG_ENV_IS_IN_SPI_FLASH */

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(768 * 1024)
#define CONFIG_ENV_SECT_SIZE		(8 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif
#else
#define CONFIG_ENV_IS_NOWHERE
#endif

#ifndef NOVENA_RAMTUNE
#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif			       /* __CONFIG_H */

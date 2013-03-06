#define DEBUG
/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6x_pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/arch/crm_regs.h>
#include <i2c.h>

#define SPD_NUMBYTES         0
#define SPD_REVISION         1
#define SPD_DRAM_DEVICE_TYPE 2
#define SPD_MODULE_TYPE      3
#define SPD_SDRAM_DENSITY_AND_BANKS 4
#define SPD_SDRAM_ADDRESSING 5
#define SPD_NOMINAL_VOLTAGE  6
#define SPD_ORGANIZATION     7
#define SPD_BUS_WIDTH        8

#define SPD_MTB_DIVIDEND     10
#define SPD_MTB_DIVISOR      11
#define SPD_TCKMIN           12

#define SPD_CL_LSB           14
#define SPD_CL_MSB           15
#define SPD_TAAMIN           16
#define SPD_TWRMIN           17
#define SPD_TRCDMIN          18
#define SPD_TRRDMIN          19
#define SPD_TRPMIN           20
#define SPD_TRAS_TRC_MSB     21
#define SPD_TRAS_LSB         22
#define SPD_TRC_LSB          23
#define SPD_TRFC_LSB         24
#define SPD_TRFC_MSB         25
#define SPD_WTRMIN           26
#define SPD_RTPMIN           27
#define SPD_TFAW_MSB         28
#define SPD_TFAW_LSB         29
#define SPD_OPTIONAL         30
#define SPD_THERMAL          31

#define SPD_VENDOR_ID_LSB    117
#define SPD_VENDOR_ID_MSB    118

#define SPD_NAME             128

#define MTB_PER_CYC          0xF  // 15 * 0.125ns per 533MHz clock cycle

struct ddr_spd {
  uint   density;
  uint   banks;
  uint   rows;
  uint   cols;
  uint   rank;
  uint   devwidth;
  uint   capacity;  // in megabits
  uint   clockrate; // in MHz
  uint   caslatency;
  uint   tAAmin;
  uint   tWRmin;
  uint   tRCDmin;
  uint   tRRDmin;
  uint   tRPmin;
  uint   tRAS;
  uint   tRC;
  uint   tRFCmin;
  uint   tWTRmin;
  uint   tRTPmin;
  uint   tFAW;
  uint   vendorID;
  u_char name[19];
};

void reg32_write(unsigned int addr, unsigned int data) {
  //  debug( "wrote %08lx to %08lx\n", data, addr );
  *((unsigned int *) addr) = (unsigned int) data;
}

unsigned int reg32_read(unsigned int addr) {
  unsigned int data;
  data = *((unsigned int *)addr);
  //  debug( "read %08lx from %08lx\n", data, addr );
  return data;
}

void reg32setbit(unsigned int addr, unsigned int bit) {
  *((unsigned int *)addr) |= (1 << bit);
}
void reg32clrbit(unsigned int addr, unsigned int bit) {
  *((unsigned int *)addr) &= ~(1 << bit);
}

//  write_level_calib(0x42);
#define MDPDC_OFFSET 0x0004
#define MDCFG0_OFFSET 0x000C
#define MDCFG1_OFFSET 0x0010
#define MDCFG2_OFFSET 0x0014
#define MAPSR_OFFSET 0x0404
#define MDREF_OFFSET 0x0020
#define MDASP_OFFSET 0x0040
#define MPZQHWCTRL_OFFSET 0x0800
#define MDSCR_OFFSET 0x001C
#define MPWLGCR_OFFSET 0x0808
#define MPWLDECTRL0_OFFSET 0x080c
#define MPWLDECTRL1_OFFSET 0x0810
#define MDCTL_OFFSET 0x0000
#define MDMISC_OFFSET 0x0018
#define MPPDCMPR1_OFFSET 0x088C
#define MPSWDAR_OFFSET 0x0894
#define MPRDDLCTL_OFFSET 0x0848
#define MPMUR_OFFSET 0x08B8
#define MPDGCTRL0_OFFSET 0x083C
#define MPDGHWST0_OFFSET 0x087C
#define MPDGHWST1_OFFSET 0x0880
#define MPDGHWST2_OFFSET 0x0884
#define MPDGHWST3_OFFSET 0x0888
#define MPDGCTRL1_OFFSET 0x0840
#define MPRDDLHWCTL_OFFSET 0x0860
#define MPWRDLCTL_OFFSET 0x0850
#define MPWRDLHWCTL_OFFSET 0x0864

#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0 0x020E05A8
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1 0x020E05B0
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2 0x020E0524
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3 0x020E051C
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4 0x020E0518
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5 0x020E050C
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6 0x020E05B8
#define IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7 0x020E05C0

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |	       \
       PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	       \
       PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	       \
       PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |	       \
       PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED	  |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define BUTTON_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1, SGTL5000 */
struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6Q_PAD_EIM_D21__GPIO_3_21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6Q_PAD_EIM_D28__GPIO_3_28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};

/* I2C2 Camera, MIPI */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6Q_PAD_KEY_COL3__GPIO_4_12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6Q_PAD_KEY_ROW3__GPIO_4_13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3, J15 - RGB connector */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6Q_PAD_GPIO_5__I2C3_SCL | PC,
		.gpio_mode = MX6Q_PAD_GPIO_5__GPIO_1_5 | PC,
		.gp = IMX_GPIO_NR(1, 5)
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_GPIO_16__I2C3_SDA | PC,
		.gpio_mode = MX6Q_PAD_GPIO_16__GPIO_7_11 | PC,
		.gp = IMX_GPIO_NR(7, 11)
	}
};

#define DISP_LINE_LEN 16

void print_i2c_dbg() {
  printf( "IADR: %04x\n", *((unsigned short *)0x21A0000) );
  printf( "IFDR: %04x\n", *((unsigned short *)0x21A0004) );
  printf( "I2CR: %04x\n", *((unsigned short *)0x21A0008) );
  printf( "I2SR: %04x\n", *((unsigned short *)0x21A000C) );
  printf( "I2DR: %04x\n", *((unsigned short *)0x21A0010) );
  printf( "clock: %08lx\n", *((unsigned int *)0x20C4070) );
}

void dram_fatal() {
  puts ("Fatal error; resetting...\n");  // reset in case it's a transient error reading SPD
  udelay (50000);				/* wait 50 ms */
  disable_interrupts();
  reset_cpu(0);
}

unsigned int mtb_to_cycles(unsigned int mtbs) {
  return (mtbs / MTB_PER_CYC) + (((mtbs % MTB_PER_CYC) > 0) ? 1 : 0);
}

int dram_init(void)
{  // init ddr3, do calibrations
  u_char	chip;
  uint	addr, alen, length;
  int	j, nbytes, linebytes;
  u_char spd[256];
  int  i;
  struct ddr_spd ddrSPD;
  unsigned short cl_support = 0;
  unsigned int cfgval = 0;
  int errorcount = 0;

  puts( "\nSPD dump:\n" );
  struct mxc_i2c_regs *i2c_regs = (struct mxc_i2c_regs *)0x21A0000;

  chip   = 0x50;
  addr   = 0x0;
  alen   = 0x1;
  length = 0x100; // length to display

  setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);

  i2c_init(100000, 0); // 100khz, second argument is not used in mx6

  nbytes = length;
  i = 0;
  do {
    unsigned char	linebuf[DISP_LINE_LEN];
    unsigned char	*cp;

    linebytes = (nbytes > DISP_LINE_LEN) ? DISP_LINE_LEN : nbytes;

    if (i2c_read(chip, addr, alen, linebuf, linebytes) != 0) {
      puts ("Error reading SPD on DDR3.\n");
      dram_fatal();
    } else {
      printf("%04x:", addr);
      cp = linebuf;
      for (j=0; j<linebytes; j++) {
	spd[i++] = *cp;
	printf(" %02x", *cp++);
	addr++;
      }
      puts ("    ");
      cp = linebuf;
      for (j=0; j<linebytes; j++) {
	if ((*cp < 0x20) || (*cp > 0x7e))
	  puts (".");
	else
	  printf("%c", *cp);
	cp++;
      }
      putc ('\n');
    }
    nbytes -= linebytes;
  } while (nbytes > 0);

  puts( "\nRaw DDR3 characteristics based on SPD:\n" );
  if( (spd[SPD_DRAM_DEVICE_TYPE] != 0xB) || (spd[SPD_MODULE_TYPE] != 3) ) {
    puts( "Unrecognized DIMM type installed\n" );
    dram_fatal();
  }
  
  if( (spd[SPD_SDRAM_DENSITY_AND_BANKS] & 0x30) != 0 ) {
    puts( "  Warning: memory has an unsupported bank size\n" );
  } else {
    puts( "  8 banks\n" );
  }
  ddrSPD.banks = 8;

  ddrSPD.density = 256 * (1 << (spd[SPD_SDRAM_DENSITY_AND_BANKS] & 0xF));
  printf( "  Individual chip density is %d Mib\n", ddrSPD.density );
  
  ddrSPD.rows = ((spd[SPD_SDRAM_ADDRESSING] & 0x38) >> 3) + 12;
  ddrSPD.cols = (spd[SPD_SDRAM_ADDRESSING] & 0x7) + 9;
  printf( "  Rows: %d, Cols: %d\n", ddrSPD.rows, ddrSPD.cols );
  
  if( spd[SPD_NOMINAL_VOLTAGE] & 0x1 ) {
    puts( "Module not operable at 1.5V, fatal error.\n" );
    dram_fatal();
  } else {
    puts( "  Supports 1.5V operation.\n" );
  }
  
  ddrSPD.rank = ((spd[SPD_ORGANIZATION] >> 3) & 0x7) + 1;
  printf( "  Module has %d rank(s)\n", ddrSPD.rank );
  
  ddrSPD.devwidth = (1 << (spd[SPD_ORGANIZATION] & 0x7)) * 4;
  printf( "  Chips have a width of %d bits\n", ddrSPD.devwidth );

  if( spd[SPD_BUS_WIDTH] != 0x3 ) {
    puts( "Unsupported device width, fatal.\n" );
    dram_fatal();
  } else {
    puts( "  Module width is 64 bits, no ECC\n" );
  }
  
  ddrSPD.capacity = (64 / ddrSPD.devwidth) * ddrSPD.rank * ddrSPD.density;
  printf( "  Module capacity is %d GiB\n", ddrSPD.capacity / 8192 );

  if( (spd[SPD_MTB_DIVIDEND] != 1) || (spd[SPD_MTB_DIVISOR] != 8) ) {
    puts( "Module has non-standard MTB for timing calculation. Doesn't mean the module is bad, just means this bootloader can't derive timing information based on the units coded in the SPD. This in, unfortunately, a fatal error. File a bug to get it fixed.\n" );
    dram_fatal();
  }
  
  switch( spd[SPD_TCKMIN] ) {
  case 0x14: ddrSPD.clockrate = 400;
    break;
  case 0x0F: ddrSPD.clockrate = 533;
    break;
  case 0x0C: ddrSPD.clockrate = 667;
    break;
  case 0x0A: ddrSPD.clockrate = 800;
    break;
  case 0x09: ddrSPD.clockrate = 933;
    break;
  case 0x08: ddrSPD.clockrate = 1067;
    break;
  default:
    if( spd[SPD_TCKMIN] <= 0xF ) {
      puts("**undecodable but sufficiently fast clock rate detected\n");
      ddrSPD.clockrate = 533;
    } else {
      puts("**undecodable but too slow clock rate detected\n");
      ddrSPD.clockrate = 400;
    }
  }
  printf( "  DDR3-%d speed rating detected\n", ddrSPD.clockrate * 2 );
  if( ddrSPD.clockrate < 533 ) {
    puts( "Fatal error: memory is too slow.\n" );
    dram_fatal();
  }
  
  //  cl_support is a bit vector with bit 0 set <-> CL=4, bit 1 set <-> CL=5, etc.
  // these are just supported rates, not the actual rate computed
  cl_support = (spd[SPD_CL_MSB] << 8) | spd[SPD_CL_LSB];
  
  ddrSPD.caslatency = mtb_to_cycles((unsigned int) spd[SPD_TAAMIN]);
  
  while( !((1 << (ddrSPD.caslatency - 4)) & cl_support) ) {
    if( ddrSPD.caslatency > 18 ) {
      puts( "Fatal error: no module-supported CAS latencies found\n" );
      dram_fatal();
    }
    ddrSPD.caslatency++;
  }
  if( ddrSPD.caslatency > 11 ) {
    puts( "Fatal error: cas latency larger than supported by i.MX6\n" );
    dram_fatal();
  }
  if( ddrSPD.caslatency < 3 ) {
    puts( "Fatal error: cas latency shorter than supported by i.MX6\n" );
    dram_fatal();
  }
  puts( "Derived optimal timing parameters, in 533MHz cycles:\n" );
  printf( "  CAS latency: %d\n", ddrSPD.caslatency );

  ddrSPD.tWRmin = mtb_to_cycles((unsigned int) spd[SPD_TWRMIN]);
  if( ddrSPD.tWRmin > 8 ) {
    puts( "  optimal tWRmin greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tWRmin = 8;
  }
  printf( "  tWRmin: %d\n", ddrSPD.tWRmin );

  ddrSPD.tRCDmin = mtb_to_cycles((unsigned int) spd[SPD_TRCDMIN]);
  if( ddrSPD.tRCDmin > 8 ) {
    puts( "  optimal tRCDmin greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tRCDmin = 8;
  }
  printf( "  tRCDmin: %d\n", ddrSPD.tRCDmin );

  ddrSPD.tRRDmin = mtb_to_cycles((unsigned int) spd[SPD_TRRDMIN]);
  if( ddrSPD.tRRDmin > 0x8 ) {
    puts( "  optimal tRRDmin greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tRRDmin = 0x8;
  }
  printf( "  tRRDmin: %d\n", ddrSPD.tRRDmin );

  ddrSPD.tRPmin = mtb_to_cycles((unsigned int) spd[SPD_TRPMIN]);
  printf( "  tRPmin: %d\n", ddrSPD.tRPmin );
  if( ddrSPD.tRPmin > 8 ) {
    puts( "  optimal tRPmin greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tRPmin = 8;
  }

  ddrSPD.tRAS = mtb_to_cycles((unsigned int) spd[SPD_TRAS_LSB] | 
			      (((unsigned int) spd[SPD_TRAS_TRC_MSB] & 0xF) << 8) );
  if( ddrSPD.tRAS > 0x20 ) {
    puts( "  optimal tRAS greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tRAS = 0x20;
  }
  printf( "  tRAS: %d\n", ddrSPD.tRAS );

  ddrSPD.tRC = mtb_to_cycles((unsigned int) spd[SPD_TRC_LSB] |
			     (((unsigned int) spd[SPD_TRAS_TRC_MSB] & 0xF0) << 4) );
  if( ddrSPD.tRC > 0x20 ) {
    puts( "  optimal tRC greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tRC = 0x20;
  }
  printf( "  tRC: %d\n", ddrSPD.tRC );

  ddrSPD.tRFCmin = mtb_to_cycles((unsigned int) spd[SPD_TRFC_LSB] | 
				 ((unsigned int) spd[SPD_TRFC_MSB]) << 8 );
  if( ddrSPD.tRFCmin > 0x100 ) {
    ddrSPD.tRFCmin = 0x100;
    puts( "  Info: derived tRFCmin exceeded max allowed value by i.MX6\n" );
  }
  printf( "  tRFCmin: %d\n", ddrSPD.tRFCmin );

  ddrSPD.tWTRmin = mtb_to_cycles((unsigned int) spd[SPD_WTRMIN]);
  if( ddrSPD.tWTRmin > 0x8 ) {
    puts( "  optimal tWTRmin greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tWTRmin = 0x8;
  }
  printf( "  tWTRmin: %d\n", ddrSPD.tWTRmin );
  
  ddrSPD.tRTPmin = mtb_to_cycles((unsigned int) spd[SPD_RTPMIN]);
  if( ddrSPD.tRTPmin > 0x8 ) {
    puts( "  optimal tRTPmin greater than supported by i.MX6, value saturated.\n" );
    ddrSPD.tRTPmin = 0x8;
  }
  printf( "  tRTPmin: %d\n", ddrSPD.tRTPmin );

  ddrSPD.tFAW = mtb_to_cycles((unsigned int) spd[SPD_TFAW_LSB] |
			      ((unsigned int) spd[SPD_TFAW_MSB]) << 8 );
  if( ddrSPD.tFAW > 0x20 ) {
    ddrSPD.tFAW = 0x20;
    puts( "  Info: derived tFAW exceeded max allowed value by i.MX6\n");
  }
  printf( "  tFAW: %d\n", ddrSPD.tFAW );

  if( spd[SPD_THERMAL] & 0x80 )
    puts( "Info: thermal sensor exists on this module\n" );
  else
    puts( "Info: no thermal sensor on-module\n" );
  
  ddrSPD.vendorID = spd[SPD_VENDOR_ID_LSB] | (spd[SPD_VENDOR_ID_MSB] << 8);
  printf( "Vendor ID: 0x%04x\n", ddrSPD.vendorID );

  for( i = 0; i < 18; i++ ) {
    ddrSPD.name[i] = spd[SPD_NAME + i];
  }
  ddrSPD.name[i] = '\0';
  printf( "Module name: %s\n", ddrSPD.name );

  puts("\nReprogramming DDR timings...\n" );

  cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET);
  printf( "Original CTL: %08x\n", cfgval );
  cfgval = 0x80000000;
  if( ddrSPD.rank == 2 ) 
    cfgval |= 0x40000000;
  cfgval |= (ddrSPD.rows - 11) << 24;
  cfgval |= (ddrSPD.cols - 9) << 20;
  cfgval |= 1 << 19; // burst length = 8
  cfgval |= 2 << 16; // data size is 64 bits
  printf( "Optimal CTL: %08x\n", cfgval );
  reg32_write(MMDC_P0_BASE_ADDR + MDCTL_OFFSET, cfgval);

  cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDASP_OFFSET);
  printf( "Original ASP: %08x\n", cfgval );
  cfgval = (ddrSPD.capacity / (256 * ddrSPD.rank)) - 1;
  printf( "Optimal ASP: %08x\n", cfgval );
  reg32_write(MMDC_P0_BASE_ADDR + MDASP_OFFSET, cfgval );

  cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCFG0_OFFSET);
  printf( "Original CFG0: %08x\n", cfgval );
  cfgval &= 0x00FFFE00;
  cfgval |= ((ddrSPD.tRFCmin - 1) << 24);
  cfgval |= ((ddrSPD.tFAW - 1) & 0x1F << 4);
  cfgval |= ddrSPD.caslatency - 3;
  printf( "Optimal CFG0: %08x\n", cfgval );
  reg32_write(MMDC_P0_BASE_ADDR + MDCFG0_OFFSET,  cfgval );

  cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCFG1_OFFSET);
  printf( "Original CFG1: %08x\n", cfgval );
  cfgval &= 0x000081FF;
  cfgval |= ((ddrSPD.tRCDmin - 1) << 29);
  cfgval |= ((ddrSPD.tRPmin - 1) << 26);
  cfgval |= ((ddrSPD.tRC - 1) << 21);
  cfgval |= ((ddrSPD.tRAS -1) << 16);
  cfgval |= ((ddrSPD.tWRmin -1) << 9);
  if( (cfgval & 0x7) + 2 < ddrSPD.caslatency ) {
    printf( "Original CFG1 tCWL shorter than supported cas latency, fixing...\n" );
    cfgval &= 0xFFFFFFF8;
    if( ddrSPD.caslatency > 7 )
      cfgval |= 0x6;
    else
      cfgval |= (ddrSPD.caslatency - 2);
  }
  printf( "Optimal CFG1: %08x\n", cfgval );
  reg32_write(MMDC_P0_BASE_ADDR + MDCFG1_OFFSET,  cfgval );

  cfgval = reg32_read(MMDC_P0_BASE_ADDR + MDCFG2_OFFSET);
  printf( "Original CFG2: %08x\n", cfgval );
  cfgval &= 0xFFFF0000;
  cfgval |= ((ddrSPD.tRTPmin - 1) << 6);
  cfgval |= ((ddrSPD.tWTRmin - 1) << 3);
  cfgval |= ((ddrSPD.tRRDmin - 1) << 0);
  printf( "Optimal CFG2: %08x\n", cfgval );
  reg32_write(MMDC_P0_BASE_ADDR + MDCFG2_OFFSET, cfgval );
    
  // write and readback some dummy data to demonstrate that ddr3 is broken
  puts("\nReference read/write test prior to tuning\n");
  do_tune_mww(NULL, 0, 1, NULL);
  do_tune_mrr(NULL, 0, 1, NULL);

  // do write (fly-by) calibration
  puts("\nFly-by calibration\n");
  errorcount = do_tune_wcal(NULL, 0, 1, NULL);
  udelay(100000);
  // let it settle in...seems it's necessary
  if( errorcount != 0 ) {
    puts( "Fly-by calibration seems to have failed. Guessing values for wcal based on rank...\n" );
    if( ddrSPD.rank == 1 ) {
      // MMDC_MPWLDECTRL0 after write level cal: 0x0047004C
      // MMDC_MPWLDECTRL1 after write level cal: 0x006B0064
      // MMDC_MPWLDECTRL0 after write level cal: 0x00690112
      // MMDC_MPWLDECTRL1 after write level cal: 0x010F0122
      reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET, 0x0047004C);
      reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET, 0x006B0064);
      reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET, 0x00690112);
      reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET, 0x010F0122);
    } else {
      // MMDC_MPWLDECTRL0 after write level cal: 0x0030003B
      // MMDC_MPWLDECTRL1 after write level cal: 0x001A005D
      // MMDC_MPWLDECTRL0 after write level cal: 0x006D0161
      // MMDC_MPWLDECTRL1 after write level cal: 0x011A013A
      reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET, 0x0030003B);
      reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET, 0x001A005D);
      reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET, 0x006D0161);
      reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET, 0x011A013A);
    }
  }

  // tune dqs delays. For some reason, has to be run twice.
  puts("\nDQS delay calibration\n");
  do_tune_delays(NULL, 0, 1, NULL);
  errorcount = do_tune_delays(NULL, 0, 1, NULL);
  if( errorcount != 0 ) {
    puts( "DQS delay calibration has failed. Guessing values for delay cal based on rank...\n" );
    if( ddrSPD.rank == 1 ) {
      // Read DQS Gating calibration
      // MPDGCTRL0 PHY0 (0x021b083c) = 0x457D060D
      // MPDGCTRL1 PHY0 (0x021b0840) = 0x06130618
      // MPDGCTRL0 PHY1 (0x021b483c) = 0x465A065A
      // MPDGCTRL1 PHY1 (0x021b4840) = 0x06680628
      // Read calibration
      // MPRDDLCTL PHY0 (0x021b0848) = 0x4B443F45
      // MPRDDLCTL PHY1 (0x021b4848) = 0x494B414D
      // Write calibration
      // MPWRDLCTL PHY0 (0x021b0850) = 0x33334333
      // MPWRDLCTL PHY1 (0x021b4850) = 0x3E2F463A
      reg32_write( 0x021b083c, 0x457D060D);
      reg32_write( 0x021b0840, 0x06130618);
      reg32_write( 0x021b483c, 0x465A065A);
      reg32_write( 0x021b4840, 0x06680628);
      reg32_write( 0x021b0848, 0x4B443F45);
      reg32_write( 0x021b4848, 0x494B414D);
      reg32_write( 0x021b0850, 0x33334333);
      reg32_write( 0x021b4850, 0x3E2F463A);
    } else {
      // Read DQS Gating calibration
      // MPDGCTRL0 PHY0 (0x021b083c) = 0x460F0625
      // MPDGCTRL1 PHY0 (0x021b0840) = 0x05590643
      // MPDGCTRL0 PHY1 (0x021b483c) = 0x467D073F
      // MPDGCTRL1 PHY1 (0x021b4840) = 0x070B0671
      // Read calibration
      // MPRDDLCTL PHY0 (0x021b0848) = 0x473F4545
      // MPRDDLCTL PHY1 (0x021b4848) = 0x47454049
      // Write calibration
      // MPWRDLCTL PHY0 (0x021b0850) = 0x43324530
      // MPWRDLCTL PHY1 (0x021b4850) = 0x3D33353F
      reg32_write( 0x021b083c, 0x460F0625);
      reg32_write( 0x021b0840, 0x05590643);
      reg32_write( 0x021b483c, 0x467D073F);
      reg32_write( 0x021b4840, 0x070B0671);
      reg32_write( 0x021b0848, 0x473F4545);
      reg32_write( 0x021b4848, 0x47454049);
      reg32_write( 0x021b0850, 0x43324530);
      reg32_write( 0x021b4850, 0x3D33353F);
    }
  }

  // confirm that the memory is working by read/write demo. Confirmation currently read out on terminal
  puts("\nReference read/write test post-tuning\n");
  do_tune_mww(NULL, 0, 1, NULL);
  do_tune_mrr(NULL, 0, 1, NULL);

  printf( "ddrSPD.capacity: %08lx\n", ddrSPD.capacity );
  printf( "Ramsize according to SPD: %08lx\n", ((ddrSPD.capacity / 8) - 256) * 1024 * 1024 );
  //  gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);
  gd->ram_size = ((ddrSPD.capacity / 8) - 256) * 1024 * 1024;
  printf( "Set gd->ram_size to %08lx\n", gd->ram_size );

  return 0;
}

iomux_v3_cfg_t uart1_pads[] = {
	MX6Q_PAD_SD3_DAT6__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6Q_PAD_SD3_DAT7__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t uart2_pads[] = {
       MX6Q_PAD_EIM_D26__UART2_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
       MX6Q_PAD_EIM_D27__UART2_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t usdhc2_pads[] = {
       MX6Q_PAD_SD2_CLK__USDHC2_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD2_CMD__USDHC2_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD2_DAT0__USDHC2_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD2_DAT1__USDHC2_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD2_DAT2__USDHC2_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD2_DAT3__USDHC2_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_GPIO_4__GPIO_1_4      | MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t usdhc3_pads[] = {
       MX6Q_PAD_SD3_CLK__USDHC3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_CMD__USDHC3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT0__USDHC3_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT1__USDHC3_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT2__USDHC3_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
       MX6Q_PAD_SD3_DAT3__USDHC3_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


iomux_v3_cfg_t enet_pads1[] = {
	MX6Q_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6Q_PAD_RGMII_RXC__GPIO_6_30		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6Q_PAD_RGMII_RD0__GPIO_6_25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6Q_PAD_RGMII_RD1__GPIO_6_27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6Q_PAD_RGMII_RD2__GPIO_6_28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6Q_PAD_RGMII_RD3__GPIO_6_29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6Q_PAD_RGMII_RX_CTL__GPIO_6_24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 42 PHY nRST */
	MX6Q_PAD_EIM_D23__GPIO_3_23		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t enet_pads2[] = {
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6Q_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

/* Button assignments for J14 */
static iomux_v3_cfg_t button_pads[] = {
	/* Menu */
	MX6Q_PAD_NANDF_D1__GPIO_2_1	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	/* Back */
	MX6Q_PAD_NANDF_D2__GPIO_2_2	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	/* Labelled Search (mapped to Power under Android) */
	MX6Q_PAD_NANDF_D3__GPIO_2_3	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	/* Home */
	MX6Q_PAD_NANDF_D4__GPIO_2_4	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	/* Volume Down */
	MX6Q_PAD_GPIO_19__GPIO_4_5	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
	/* Volume Up */
	MX6Q_PAD_GPIO_18__GPIO_7_13	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	gpio_direction_output(IMX_GPIO_NR(3, 23), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 30), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	gpio_direction_output(IMX_GPIO_NR(6, 24), 1);

	/* Need delay 10ms according to KSZ9021 spec */
	udelay(1000 * 10);
	gpio_set_value(IMX_GPIO_NR(3, 23), 1);

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

iomux_v3_cfg_t usb_pads[] = {
	MX6Q_PAD_GPIO_17__GPIO_7_12 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
       imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

	/* Reset USB hub */
	gpio_direction_output(IMX_GPIO_NR(7, 12), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(7, 12), 1);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
       {USDHC3_BASE_ADDR},
       {USDHC2_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret;

	if (cfg->esdhc_base == USDHC3_BASE_ADDR) {
		gpio_direction_input(IMX_GPIO_NR(7, 0));
		ret = 1; // there is no CD for a microSD card, and if we booted this baby is there.
	} else {
		gpio_direction_input(IMX_GPIO_NR(1, 4));
		ret = !gpio_get_value(IMX_GPIO_NR(1, 4));
	}

       return ret;
}

int board_mmc_init(bd_t *bis)
{
       s32 status = 0;
       u32 index = 0;

       for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
	       switch (index) {
	       case 0:
		       imx_iomux_v3_setup_multiple_pads(
			       usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		       break;
	       case 1:
		       imx_iomux_v3_setup_multiple_pads(
			       usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
		       break;
	       default:
		       printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
		       return status;
	       }

	       status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
       }

       return status;
}
#endif

u32 get_board_rev(void)
{
	return 0x63000 ;
}

#ifdef CONFIG_MXC_SPI
iomux_v3_cfg_t ecspi1_pads[] = {
	/* SS1 */
	MX6Q_PAD_EIM_D19__GPIO_3_19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

void setup_spi(void)
{
	gpio_direction_output(CONFIG_SF_DEFAULT_CS, 1);
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));
}
#endif

int board_phy_config(struct phy_device *phydev)
{
#ifdef CONFIG_FEC_MXC
	/* min rx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
	/* min tx data delay */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
	/* max rx/tx clock delay, min rx/tx control */
	ksz9021_phy_extended_write(phydev,
			MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
	if (phydev->drv->config)
		phydev->drv->config(phydev);
#endif
	return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}

static void setup_buttons(void)
{
	imx_iomux_v3_setup_multiple_pads(button_pads,
					 ARRAY_SIZE(button_pads));
}

#ifdef CONFIG_CMD_SATA

int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;

	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);

	return 0;
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)
static struct fb_videomode videomodes[] = {{
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT, /* a.k.a. LVDS */
		.vmode          = FB_VMODE_NONINTERLACED
	}, {
		.name           = "wsvga-lvds",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT, /* a.k.a. LVDS */
		.vmode          = FB_VMODE_NONINTERLACED
	}, {
		.name           = "wvga-rgb",
		.refresh        = 57,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 37037,
		.left_margin    = 40,
		.right_margin   = 60,
		.upper_margin   = 10,
		.lower_margin   = 10,
		.hsync_len      = 20,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} };

#define IS_LVDS(m) (FB_SYNC_EXT & (m).sync)

struct i2c_display {
	int bus;
	int addr;
	char const *name;
};

static struct i2c_display const i2c_displays[] = {
{
	.bus		= 2,
	.addr		= 0x4,
	.name           = "Hannstar-XGA",
}, {
	.bus		= 2,
	.addr		= 0x38,
	.name           = "wsvga-lvds",
}, {
	.bus		= 2,
	.addr		= 0x48,
	.name           = "wvga-rgb",
} };

static iomux_v3_cfg_t rgb_pads[] = {
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,
	MX6Q_PAD_DI0_PIN4__GPIO_4_20,
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
};

static iomux_v3_cfg_t backlight_pads[] = {
	/* Backlight on RGB connector: J15 */
	MX6Q_PAD_SD1_DAT3__GPIO_1_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define RGB_BACKLIGHT_GP IMX_GPIO_NR(1, 21)

	/* Backlight on LVDS connector: J6 */
	MX6Q_PAD_SD1_CMD__GPIO_1_18 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(1, 18)
};

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(i2c_displays); i++) {
			struct i2c_display const *dev = i2c_displays+i;
			if ((0 == i2c_set_bus_num(dev->bus))
			     &&
			    (0 == i2c_probe(dev->addr))) {
				panel = dev->name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = videomodes[0].name;
			printf("No panel detected: default to %s\n", panel);
		}
	}
	for (i = 0; i < ARRAY_SIZE(videomodes); i++) {
		if (!strcmp(panel, videomodes[i].name))
			break;
	}
	if (i < ARRAY_SIZE(videomodes)) {
		ret = ipuv3_fb_init(videomodes+i, 0, IPU_PIX_FMT_RGB666);
		if (!ret) {
			if (IS_LVDS(videomodes[i]))
				gpio_direction_output(LVDS_BACKLIGHT_GP, 1);
			else {
				imx_iomux_v3_setup_multiple_pads(
					rgb_pads,
					 ARRAY_SIZE(rgb_pads));
				gpio_direction_output(RGB_BACKLIGHT_GP, 1);
			}
			printf("Display: %s (%ux%u)\n",
			       videomodes[i].name,
			       videomodes[i].xres,
			       videomodes[i].yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       videomodes[i].name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		ret = -EINVAL;
	}
	return (0 != ret);
}

void lcd_iomux(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg;

	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=   MXC_CCM_CCGR3_IPU1_IPU_DI0_OFFSET
		|MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set PFD1_FRAC to 0x13 == 455 MHz (480*18)/0x13 */
	writel(ANATOP_PFD_480_PFD1_FRAC_MASK, &anatop->pfd_480_clr);
	writel(0x13<<ANATOP_PFD_480_PFD1_FRAC_SHIFT, &anatop->pfd_480_set);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 |MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      |(3<<MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK
		|MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK
		|MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET)
	      |(CHSCCDR_PODF_DIVIDE_BY_3
		<<MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
	      |(CHSCCDR_IPU_PRE_CLK_540M_PFD
		<<MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~IOMUXC_GPR3_LVDS0_MUX_CTL_MASK)
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);

	/* backlights off until needed */
	imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_input(LVDS_BACKLIGHT_GP);
	gpio_direction_input(RGB_BACKLIGHT_GP);
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_buttons();
#if defined(CONFIG_VIDEO_IPUV3)
	lcd_iomux();
#endif
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
       /* address of boot parameters */
       gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

       return 0;
}

int checkboard(void)
{
       puts("Board: Novena Quattuor\n");

       return 0;
}

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
	{"back",	IMX_GPIO_NR(2, 2),	'B'},
	{"home",	IMX_GPIO_NR(2, 4),	'H'},
	{"menu",	IMX_GPIO_NR(2, 1),	'M'},
	{"search",	IMX_GPIO_NR(2, 3),	'S'},
	{"volup",	IMX_GPIO_NR(7, 13),	'V'},
	{"voldown",	IMX_GPIO_NR(4, 5),	'v'},
};

/*
 * generate a null-terminated string containing the buttons pressed
 * returns number of keys pressed
 */
static int read_keys(char *buf)
{
	int i, numpressed = 0;
	for (i = 0; i < ARRAY_SIZE(buttons); i++) {
		if (!gpio_get_value(buttons[i].gpnum))
			buf[numpressed++] = buttons[i].ident;
	}
	buf[numpressed] = '\0';
	return numpressed;
}

static int do_kbd(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char envvalue[ARRAY_SIZE(buttons)+1];
	int numpressed = read_keys(envvalue);
	setenv("keybd", envvalue);
	return numpressed == 0;
}

U_BOOT_CMD(
	kbd, 1, 1, do_kbd,
	"Tests for keypresses, sets 'keybd' environment variable",
	"Returns 0 (true) to shell if key is pressed."
);

#ifdef CONFIG_PREBOOT
static char const kbd_magic_prefix[] = "key_magic";
static char const kbd_command_prefix[] = "key_cmd";

static void preboot_keys(void)
{
	int numpressed;
	char keypress[ARRAY_SIZE(buttons)+1];
	numpressed = read_keys(keypress);
	if (numpressed) {
		char *kbd_magic_keys = getenv("magic_keys");
		char *suffix;
		/*
		 * loop over all magic keys
		 */
		for (suffix = kbd_magic_keys; *suffix; ++suffix) {
			char *keys;
			char magic[sizeof(kbd_magic_prefix) + 1];
			sprintf(magic, "%s%c", kbd_magic_prefix, *suffix);
			keys = getenv(magic);
			if (keys) {
				if (!strcmp(keys, keypress))
					break;
			}
		}
		if (*suffix) {
			char cmd_name[sizeof(kbd_command_prefix) + 1];
			char *cmd;
			sprintf(cmd_name, "%s%c", kbd_command_prefix, *suffix);
			cmd = getenv(cmd_name);
			if (cmd) {
				setenv("preboot", cmd);
				return;
			}
		}
	}
}
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0",	MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{"mmc1",	MAKE_CFGVAL(0x40, 0x38, 0x00, 0x00)},
	{NULL,		0},
};
#endif

int misc_init_r(void)
{
#ifdef CONFIG_PREBOOT
	preboot_keys();
#endif

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	return 0;
}



///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

#define DISP_LINE_LEN 16
int do_tune_md ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong	addr, length;
	int	size;
	int rc = 0;

	//////// globals broken
	uint	dp_last_addr = 0, dp_last_size = 0;
	uint	dp_last_length = 0x40;
	uint	mm_last_addr = 0, mm_last_size = 0;
	ulong	base_address = 0;
	////////

	/* We use the last specified parameters, unless new ones are
	 * entered.
	 */
	addr = dp_last_addr;
	size = dp_last_size;
	length = dp_last_length;

	if (argc < 2)
		return CMD_RET_USAGE;

	if ((flag & CMD_FLAG_REPEAT) == 0) {
		/* New command specified.  Check for a size specification.
		 * Defaults to long if no or incorrect specification.
		 */
		if ((size = cmd_get_data_size(argv[0], 4)) < 0)
			return 1;

		/* Address is specified since argc > 1
		*/
		addr = simple_strtoul(argv[1], NULL, 16);
		addr += base_address;

		/* If another parameter, it is the length to display.
		 * Length is the number of objects, not number of bytes.
		 */
		if (argc > 2)
			length = simple_strtoul(argv[2], NULL, 16);
	}

	/* Print the lines. */
	print_buffer(addr, (void*)addr, size, length, DISP_LINE_LEN/size);
	addr += size*length;

	dp_last_addr = addr;
	dp_last_length = length;
	dp_last_size = size;
	return (rc);
}

int do_tune_mw ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong	addr, writeval, count;
	int	size;

	//////// globals broken
	uint	dp_last_addr = 0, dp_last_size = 0;
	uint	dp_last_length = 0x40;
	uint	mm_last_addr = 0, mm_last_size = 0;
	ulong	base_address = 0;
	////////

	if ((argc < 3) || (argc > 4))
		return CMD_RET_USAGE;

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 1)
		return 1;

	/* Address is specified since argc > 1
	*/
	addr = simple_strtoul(argv[1], NULL, 16);
	addr += base_address;

	/* Get the value to write.
	*/
	writeval = simple_strtoul(argv[2], NULL, 16);

	/* Count ? */
	if (argc == 4) {
		count = simple_strtoul(argv[3], NULL, 16);
	} else {
		count = 1;
	}

	while (count-- > 0) {
		if (size == 4)
			*((ulong  *)addr) = (ulong )writeval;
		else if (size == 2)
			*((ushort *)addr) = (ushort)writeval;
		else
			*((u_char *)addr) = (u_char)writeval;
		addr += size;
	}
	return 0;
}

int do_tune_mw2 ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong	addr, writeval, count;
	int	size;

	//////// globals broken
	uint	dp_last_addr = 0, dp_last_size = 0;
	uint	dp_last_length = 0x40;
	uint	mm_last_addr = 0, mm_last_size = 0;
	ulong	base_address = 0;
	////////

	if ((argc < 3))
		return CMD_RET_USAGE;

	/* Check for size specification.
	*/
	if ((size = cmd_get_data_size(argv[0], 4)) < 1)
		return 1;

	/* Address is specified since argc > 1
	*/
	addr = simple_strtoul(argv[1], NULL, 16);
	addr += base_address;

	/* Get the value to write.
	*/
	writeval = simple_strtoul(argv[2], NULL, 16);

	count = 1;

	while (count-- > 0) {
		if (size == 4)
			*((ulong  *)addr) = (ulong )writeval;
		else if (size == 2)
			*((ushort *)addr) = (ushort)writeval;
		else
			*((u_char *)addr) = (u_char)writeval;
		addr += size;
	}
	return 0;
}

#define BIT(n,x) ( ( (x) >> (n) ) & 1 )

int do_tune_mww ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
  ulong	addr, writeval, count, bank, feedback, sum1;
	int	size;

	//////// globals broken
	uint	dp_last_addr = 0, dp_last_size = 0;
	uint	dp_last_length = 0x40;
	uint	mm_last_addr = 0, mm_last_size = 0;
	ulong	base_address = 0x10000000;
	////////

	if( argc > 1 )
	  bank = simple_strtoul(argv[1], NULL, 16);
	else
	  bank = 0;

	/* Address is specified since argc > 1
	*/
	if( bank == 0 )
	  addr = base_address;
	else
	  addr = base_address + 0x90000000;
	
	printf( "write starting at %08lx\n", addr );

	/* Get the value to write.
	*/
	writeval = 0x0;

	count = 0x20000;
	size = 2;
	sum1 = 0;

	while (count-- > 0) {
	  feedback = (BIT(14,writeval) == BIT(13,writeval));
	  writeval = (writeval<<1) + feedback;
	  writeval &= 0x7FFF;
	  *((ushort  *)addr) = (ushort )writeval;
	  sum1 += (ushort )writeval;
	  addr += size;
	}
	printf( "checksum: %08lx\n", sum1 );
	return 0;
}

int do_tune_mrr ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
  ulong	addr, writeval, count, bank, feedback, sum1, sum2;
	int	size;

	//////// globals broken
	uint	dp_last_addr = 0, dp_last_size = 0;
	uint	dp_last_length = 0x40;
	uint	mm_last_addr = 0, mm_last_size = 0;
	ulong	base_address = 0x10000000;
	////////

	if( argc > 1 )
	  bank = simple_strtoul(argv[1], NULL, 16);
	else
	  bank = 0;

	/* Address is specified since argc > 1
	*/
	if( bank == 0 )
	  addr = base_address;
	else
	  addr = base_address + 0x90000000;
	printf( "read starting at %08lx\n", addr );

	/* Get the value to write.
	*/
	writeval = 0x0;
	size = 2;

	sum1 = 0;
	sum2 = 0;
	count = 0x20000;
	feedback = 0;
	while (count-- > 0) {
	  feedback = (BIT(14,writeval) == BIT(13,writeval));
	  writeval = (writeval<<1) + feedback;
	  writeval &= 0x7FFF;
	  sum1 += (ushort )writeval;
	  sum2 += *((ushort  *)addr);
	  addr += size;
	}
	printf("computed: %08lx, readback: %08lx\n", sum1, sum2);
	return 0;
}


/*
 * Perform a memory test. A more complete alternative test can be
 * configured using CONFIG_SYS_ALT_MEMTEST. The complete test loops until
 * interrupted by ctrl-c or by a failure of one of the sub-tests.
 */
int do_tune_mtest (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	vu_long	*addr, *start, *end;
	ulong	val;
	ulong	readback;
	ulong	errs = 0;
	int iterations = 1;
	int iteration_limit;

	vu_long *dummy = 0;	/* yes, this is address 0x0, not NULL */
	int	j;

	//////// globals broken
	uint	dp_last_addr = 0, dp_last_size = 0;
	uint	dp_last_length = 0x40;
	uint	mm_last_addr = 0, mm_last_size = 0;
	ulong	base_address = 0;
	////////

	static const ulong bitpattern[] = {
		0x00000001,	/* single bit */
		0x00000003,	/* two adjacent bits */
		0x00000007,	/* three adjacent bits */
		0x0000000F,	/* four adjacent bits */
		0x00000005,	/* two non-adjacent bits */
		0x00000015,	/* three non-adjacent bits */
		0x00000055,	/* four non-adjacent bits */
		0xaaaaaaaa,	/* alternating 1/0 */
	};

	ulong	incr;
	ulong	pattern;

	if (argc > 1)
		start = (ulong *)simple_strtoul(argv[1], NULL, 16);
	else
		start = (ulong *)CONFIG_SYS_MEMTEST_START;

	if (argc > 2)
		end = (ulong *)simple_strtoul(argv[2], NULL, 16);
	else
		end = (ulong *)(CONFIG_SYS_MEMTEST_END);

	if (argc > 3)
		pattern = (ulong)simple_strtoul(argv[3], NULL, 16);
	else
		pattern = 0;

	if (argc > 4)
		iteration_limit = (ulong)simple_strtoul(argv[4], NULL, 16);
	else
		iteration_limit = 0;

	incr = 1;
	for (;;) {
		if (ctrlc()) {
			putc ('\n');
			return 1;
		}

		if (iteration_limit && iterations > iteration_limit) {
			printf("Tested %d iteration(s) with %lu errors.\n",
				iterations-1, errs);
			return errs != 0;
		}
		++iterations;

		printf ("\rPattern %08lX  Writing..."
			"%12s"
			"\b\b\b\b\b\b\b\b\b\b",
			pattern, "");

		for (addr=start,val=pattern; addr<end; addr++) {
		  //			WATCHDOG_RESET();
			*addr = val;
			val  += incr;
		}

		puts ("Reading...");

		for (addr=start,val=pattern; addr<end; addr++) {
		  //			WATCHDOG_RESET();
			readback = *addr;
			if (readback != val) {
				printf ("\nMem error @ 0x%08X: "
					"found %08lX, expected %08lX\n",
					(uint)(uintptr_t)addr, readback, val);
				errs++;
				if (ctrlc()) {
					putc ('\n');
					return 1;
				}
			}
			val += incr;
		}

		/*
		 * Flip the pattern each time to make lots of zeros and
		 * then, the next time, lots of ones.  We decrement
		 * the "negative" patterns and increment the "positive"
		 * patterns to preserve this feature.
		 */
		if(pattern & 0x80000000) {
			pattern = -pattern;	/* complement & increment */
		}
		else {
			pattern = ~pattern;
		}
		incr = -incr;
	}
	return 0;	/* not reached */
}

int do_tune_wcal(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
  int temp1, temp2, temp3, dummy;
  int errorcount = 0;
  int ddr_mr1 = 0x04;
  int wldel0 = 0;
  int wldel1 = 0;
  int wldel2 = 0;
  int wldel3 = 0;
  int wldel4 = 0;
  int wldel5 = 0;
  int wldel6 = 0;
  int wldel7 = 0;
  int i;
  int withprint = 1;
  int ldectrl[4];

  if (argc > 1)
    ddr_mr1 = (int) simple_strtoul(argv[1], NULL, 16);
  else
    ddr_mr1 = 0x04;

  if (argc > 2) {
    withprint = (int)simple_strtoul(argv[2], NULL, 16) & 0x3;
  }
  if (argc > 3) {
    wldel0 = (int)simple_strtoul(argv[3], NULL, 16) & 0x3;
    wldel1 = (int)simple_strtoul(argv[4], NULL, 16) & 0x3;
    wldel2 = (int)simple_strtoul(argv[5], NULL, 16) & 0x3;
    wldel3 = (int)simple_strtoul(argv[6], NULL, 16) & 0x3;
    wldel4 = (int)simple_strtoul(argv[7], NULL, 16) & 0x3;
    wldel5 = (int)simple_strtoul(argv[8], NULL, 16) & 0x3;
    wldel6 = (int)simple_strtoul(argv[9], NULL, 16) & 0x3;
    wldel7 = (int)simple_strtoul(argv[10], NULL, 16) & 0x3;

    reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET,
		(reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET) & 0xF9FFF9FF) |
		(wldel0 << 9) | (wldel1 << 25));

    reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET,
		(reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET) & 0xF9FFF9FF) |
		(wldel2 << 9) | (wldel3 << 25));

    reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET,
		(reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET) & 0xF9FFF9FF) |
		(wldel4 << 9) | (wldel5 << 25));

    reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET,
		(reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET) & 0xF9FFF9FF) |
		(wldel6 << 9) | (wldel7 << 25));

    debug("MMDC_MPWLDECTRL0 before write level cal: 0x%08X\n",
	   reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET));
    debug("MMDC_MPWLDECTRL1 before write level cal: 0x%08X\n",
	   reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET));
    debug("MMDC_MPWLDECTRL0 before write level cal: 0x%08X\n",
	   reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET));
    debug("MMDC_MPWLDECTRL1 before write level cal: 0x%08X\n",
	   reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET));
  }

  // stash old values in case calibration fails, we need to restore them
  ldectrl[0] = reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET);
  ldectrl[1] = reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET);
  ldectrl[2] = reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET);
  ldectrl[3] = reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET);

  // disable DDR logic power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) & 0xffff00ff);
  // disable Adopt power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) | 0x1);

  debug("Start write leveling calibration \n");
  // 2. disable auto refresh and ZQ calibration
  // before proceeding with Write Leveling calibration
  temp1 = reg32_read(MMDC_P0_BASE_ADDR + MDREF_OFFSET);
  reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), 0x0000C000);
  temp2 = reg32_read(MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET);
  reg32_write((MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET), temp2 & ~(0x3));

  // 3. increase walat and ralat to maximum
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 6); //set RALAT to max
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 7);
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 8);
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 16); //set WALAT to max
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 17);

  reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 6); //set RALAT to max
  reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 7);
  reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 8);
  reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 16); //set WALAT to max
  reg32setbit((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), 17);

  // 4 & 5. Configure the external DDR device to enter write leveling mode
  // through Load Mode Register command
  // Register setting:
  // Bits[31:16] MR1 value (0x0080 write leveling enable)
  // Bit[9] set WL_EN to enable MMDC DQS output
  // Bits[6:4] set CMD bits for Load Mode Register programming
  // Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
  reg32_write(MMDC_P0_BASE_ADDR + MDSCR_OFFSET,0x00808231);

  // 6. Activate automatic calibration by setting MPWLGCR[HW_WL_EN]
  reg32_write(MMDC_P0_BASE_ADDR + MPWLGCR_OFFSET,0x00000001);

  // 7. Upon completion of this process the MMDC de-asserts the MPWLGCR[HW_WL_EN]
  dummy = 0;
  while (reg32_read(MMDC_P0_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000001) {
    if( withprint ) 
      printf( "." );
  }
 // printf( "\n" );

  // 8. check for any errors: check both PHYs for x64 configuration, if x32, check only PHY0
  if ((reg32_read(MMDC_P0_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000F00) ||
      (reg32_read(MMDC_P1_BASE_ADDR + MPWLGCR_OFFSET) & 0x00000F00))
    {
      errorcount++;
    }
  debug("Write leveling calibration completed, errcount: %d\n", errorcount);

  // check to see if cal failed
  if( (reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET)) == 0x001F001F &&
      (reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET)) == 0x001F001F &&
      (reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET)) == 0x001F001F &&
      (reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET)) == 0x001F001F ) {
    debug( "Cal seems to have soft-failed due to memory not supporting write leveling on all channels. Restoring original write leveling values.\n" );
    reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET, ldectrl[0]);
    reg32_write(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET, ldectrl[1]);
    reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET, ldectrl[2]);
    reg32_write(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET, ldectrl[3]);
    errorcount++;
  }
  
  // User should issue MRS command to exit write leveling mode
  // through Load Mode Register command
  // Register setting:
  // Bits[31:16] MR1 value "ddr_mr1" value from initialization
  // Bit[9] clear WL_EN to disable MMDC DQS output
  // Bits[6:4] set CMD bits for Load Mode Register programming
  // Bits[2:0] set CMD_BA to 0x1 for DDR MR1 programming
  reg32_write( MMDC_P0_BASE_ADDR + MDSCR_OFFSET,((ddr_mr1 << 16)+0x8031));
  // re-enable to auto refresh and zq cal
  reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), temp1);
  reg32_write((MMDC_P0_BASE_ADDR + MPZQHWCTRL_OFFSET), temp2);
  reg32_write((MMDC_P1_BASE_ADDR + MPZQHWCTRL_OFFSET), temp3);
  debug("MMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
	 reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL0_OFFSET));
  debug("MMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
	 reg32_read(MMDC_P0_BASE_ADDR + MPWLDECTRL1_OFFSET));
  debug("MMDC_MPWLDECTRL0 after write level cal: 0x%08X\n",
	 reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL0_OFFSET));
  debug("MMDC_MPWLDECTRL1 after write level cal: 0x%08X\n",
	 reg32_read(MMDC_P1_BASE_ADDR + MPWLDECTRL1_OFFSET));
  // enable DDR logic power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) | 0x00005500);
  // enable Adopt power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) & 0xfffffff7);
  reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET),0); //clear CON_REQ
  return errorcount;
}


int modify_dg_result(int reg_st0, int reg_st1, int reg_ctrl)
{
  // DQS gating absolute offset should be modified from reflecting (HW_DG_LOWx + HW_DG_UPx)/2
  // to reflecting (HW_DG_UPx - 0x80)
  int dg_tmp_val0,dg_tmp_val1, dg_tmp_val2;
  int dg_dl_abs_offset0, dg_dl_abs_offset1;
  int dg_hc_del0, dg_hc_del1;
  dg_tmp_val0 = ((reg32_read(reg_st0) & 0x07ff0000) >>16) - 0xc0;
  dg_tmp_val1 = ((reg32_read(reg_st1) & 0x07ff0000) >>16) - 0xc0;
  dg_dl_abs_offset0 = dg_tmp_val0 & 0x7f;
  dg_hc_del0 = (dg_tmp_val0 & 0x780) << 1;
  dg_dl_abs_offset1 = dg_tmp_val1 & 0x7f;
  dg_hc_del1 = (dg_tmp_val1 & 0x780) << 1;
  dg_tmp_val2 = dg_dl_abs_offset0 + dg_hc_del0 + ((dg_dl_abs_offset1 +
						   dg_hc_del1) << 16);
  reg32_write((reg_ctrl),
	      reg32_read((reg_ctrl)) & 0xf0000000);
  reg32_write((reg_ctrl),
	      reg32_read((reg_ctrl)) & 0xf0000000);
  reg32_write((reg_ctrl),
	      reg32_read((reg_ctrl)) | dg_tmp_val2);
}

int do_tune_delays(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
  int temp1;
  int g_error_read_dqs_gating;
  int g_error_read_cal;
  int g_error_write_cal;
  int data_bus_size;
  int temp_ref;
  int cs0_enable = 0;
  int cs1_enable = 0;
  int cs0_enable_initial = 0;
  int cs1_enable_initial = 0;

  // int PDDWord = 0x55aaaa55; // original values, these work, but can getslightly better below
  int PDDWord = 0x00FFFF00; // best so far, place into MPPDCMPR1
  int errorcount = 0;
  int withprint = 1;
  unsigned int initdelay = 0x40404040;

  if (argc > 1)
    withprint = (int) simple_strtoul(argv[1], NULL, 16);

  if (argc > 2 )
    initdelay = (int) simple_strtoul(argv[2], NULL, 16);

  // check to see which chip selects are enabled
  cs0_enable_initial = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET) &
			0x80000000) >> 31;
  cs1_enable_initial = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET) &
			0x40000000) >> 30;
  debug( "init cs0: %d cs1: %d\n", cs0_enable_initial, cs1_enable_initial );
  // disable DDR logic power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) & 0xffff00ff);

  // disable Adopt power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) | 0x1);

  // set the device ODT during read:
  // reg32_write((MMDC_P0_BASE_ADDR + MPODTCTRL_OFFSET),
  // reg32_read((MMDC_P0_BASE_ADDR + MPODTCTRL_OFFSET)) |  0x8);

  //set DQS pull ups
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6) | 0x7000);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7) | 0x7000);
  temp1 = reg32_read(MMDC_P0_BASE_ADDR + MDMISC_OFFSET);
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 6); //set RALAT to max
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 7);
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 8);
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 16); //set WALAT to max
  reg32setbit((MMDC_P0_BASE_ADDR + MDMISC_OFFSET), 17);

  // disable auto refresh
  // before proceeding with calibration
  temp_ref = reg32_read(MMDC_P0_BASE_ADDR + MDREF_OFFSET);
  reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), 0x0000C000);
  // per the ref manual, issue one refresh cycle MDSCR[CMD]= 0x2, this also sets the CON_REQ bit.
  if (cs0_enable_initial == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x00008020);
  if (cs1_enable_initial == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x00008028);
  // poll to make sure the con_ack bit was asserted
  while (!(reg32_read((MMDC_P0_BASE_ADDR + MDSCR_OFFSET)) & 0x00004000)) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // check MDMISC register CALIB_PER_CS to see which CS calibration is targeted to (under normal
  // cases, it should be cleared as this is the default value, indicating calibration is directed to CS0).
  // Disable the other chip select not being target for calibration to avoid any potential issues
  // This will get re-enabled at end of calibration
  if ((reg32_read(MMDC_P0_BASE_ADDR + MDMISC_OFFSET) & 0x00100000) == 0) {
    reg32clrbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 30); // clear SDE_1
  } else {
    reg32clrbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 31); // clear SDE_0
  }
  // check to see which chip selects are now enabled for the remainder of the calibration
  cs0_enable = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET) & 0x80000000) >> 31;
  cs1_enable = (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET) & 0x40000000) >> 30;
  debug( "cal cs0: %d cs1: %d\n", cs0_enable, cs1_enable );
  // check to see what is the data bus size:
  data_bus_size= (reg32_read(MMDC_P0_BASE_ADDR + MDCTL_OFFSET) & 0x30000) >> 16;
  debug( "db size: %d\n", data_bus_size );
  // Issue the Precharge-All command to the DDR device for both chip selects
  // Note, CON_REQ bit should also remain set
  // If only using one chip select, then precharge only the desired chip select
  if (cs0_enable == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008050); // CS0
  if (cs1_enable == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008058); // CS1

  // Write the pre-defined value into MPPDCMPR1
  reg32_write((MMDC_P0_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);
  // Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
  // in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
  // write access.
  reg32setbit((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET), 0);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET)) & 0x00000001) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );

  /* this is for read delay cal. why is it here in DQS gating? */
  // Set the RD_DL_ABS_OFFSET# bits to their default values (will be calibrated later in
  // the read delay-line calibration)
  // Both PHYs for x64 configuration, if x32, do only PHY0
  reg32_write((MMDC_P0_BASE_ADDR + MPRDDLCTL_OFFSET), 0x40404040);
  if (data_bus_size == 0x2) {
    reg32_write((MMDC_P1_BASE_ADDR + MPRDDLCTL_OFFSET), 0x40404040);
  }
  //Force a measurment, for previous delay setup to take effect:
  reg32_write((MMDC_P0_BASE_ADDR + MPMUR_OFFSET), 0x800);
  if (data_bus_size == 0x2) {
    reg32_write((MMDC_P1_BASE_ADDR + MPMUR_OFFSET), 0x800);
  }
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // Read DQS Gating calibration
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  debug("Starting DQS gating calibration...\n");
  // Reset the read data FIFOs (two resets); only need to issue reset to PHY0 since in x64
  // mode, the reset will also go to PHY1
  // read data FIFOs reset1
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // read data FIFOs reset2
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // Start the automatic read DQS gating calibration process by asserting MPDGCTRL0[HW_DG_EN]
  // and MPDGCTRL0[DG_CMP_CYC] and then poll MPDGCTRL0[HW_DG_EN]] until this bit clears to
  // indicate completion.
  // Also, ensure that MPDGCTRL0[HW_DG_ERR] is clear to indicate no errors were seen during
  // calibration.
  // Set bit 30: chooses option to wait 32 cycles instead of 16 before comparing read data
  reg32setbit((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET), 30);
  // Set bit 28 to start automatic read DQS gating calibration
  reg32setbit((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET), 28);
  // Poll for completion
  // MPDGCTRL0[HW_DG_EN] should be 0
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x10000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // Check to see if any errors were encountered during calibration
  // (check MPDGCTRL0[HW_DG_ERR])
  // check both PHYs for x64 configuration, if x32, check only PHY0
  if (data_bus_size == 0x2) {
    if ((reg32_read(MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x00001000) ||
	(reg32_read(MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x00001000)) {
      errorcount++; // increment the errorcount variable
      g_error_read_dqs_gating = 1; // set the g_error_read_dqs_gating
    }
  } else {
    if (reg32_read(MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET) & 0x00001000) {
      errorcount++; // increment the errorcount variable
      g_error_read_dqs_gating = 1; // set the g_error_read_dqs_gating
    }
  }
  debug( "errorcount: %d\n", errorcount );
  // DQS gating absolute offset should be modified from reflecting (HW_DG_LOWx + HW_DG_UPx)/2
  // to reflecting (HW_DG_UPx - 0x80)
  modify_dg_result(MMDC_P0_BASE_ADDR + MPDGHWST0_OFFSET,
		   MMDC_P0_BASE_ADDR + MPDGHWST1_OFFSET,
		   MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET);
  modify_dg_result(MMDC_P0_BASE_ADDR + MPDGHWST2_OFFSET,
		   MMDC_P0_BASE_ADDR + MPDGHWST3_OFFSET,
		   MMDC_P0_BASE_ADDR + MPDGCTRL1_OFFSET);
  if (data_bus_size == 0x2) {
    modify_dg_result((MMDC_P1_BASE_ADDR + MPDGHWST0_OFFSET),
		     (MMDC_P1_BASE_ADDR + MPDGHWST1_OFFSET),
		     (MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET));
    modify_dg_result((MMDC_P1_BASE_ADDR + MPDGHWST2_OFFSET),
		     (MMDC_P1_BASE_ADDR + MPDGHWST3_OFFSET),
		     (MMDC_P1_BASE_ADDR + MPDGCTRL1_OFFSET));
  }
  debug("DQS gating calibration completed, hit enter to proceed.\n");
  //  getc();



  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // Read delay Calibration
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  debug("Starting read calibration...\n");
  // Reset the read data FIFOs (two resets); only need to issue reset to PHY0 since in x64
  // mode, the reset will also go to PHY1
  // read data FIFOs reset1
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // read data FIFOs reset2
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // 4. Issue the Precharge-All command to the DDR device for both chip selects
  // If only using one chip select, then precharge only the desired chip select
  if (cs0_enable == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008050); // CS0
  while( !(reg32_read(MMDC_P0_BASE_ADDR + MDSCR_OFFSET) & 0x4000) ) {
    printf( "x" );
  }
  if (cs1_enable == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008058); // CS1
  while( !(reg32_read(MMDC_P0_BASE_ADDR + MDSCR_OFFSET) & 0x4000) ) {
    printf( "x" );
  }

  ////////////// 5. 6. 7. set the pre-defined word ///////////////
  reg32_write((MMDC_P0_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);
  // Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
  // in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
  // write access.
  reg32setbit((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET), 0);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET)) & 0x00000001) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );

  //// 8. set initial delays to center up dq in clock
  reg32_write((MMDC_P0_BASE_ADDR + MPRDDLCTL_OFFSET), initdelay);
  if (data_bus_size == 0x2) {
    reg32_write((MMDC_P1_BASE_ADDR + MPRDDLCTL_OFFSET), initdelay);
  }
  debug( "intdel0: %08lx / intdel1: %08lx\n", 
	  reg32_read(MMDC_P0_BASE_ADDR + MPRDDLCTL_OFFSET),
	  reg32_read(MMDC_P1_BASE_ADDR + MPRDDLCTL_OFFSET));

  // 9. Read delay-line calibration
  // Start the automatic read calibration process by asserting MPRDDLHWCTL[HW_RD_DL_EN]
  reg32_write((MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET), 0x00000030);

  // 10. poll for completion
  // MMDC indicates that the write data calibration had finished by setting
  // MPRDDLHWCTL[HW_RD_DL_EN] = 0
  // Also, ensure that no error bits were set
  while (reg32_read((MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET)) & 0x00000010) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // check both PHYs for x64 configuration, if x32, check only PHY0
  if (data_bus_size == 0x2) {
    if ((reg32_read(MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x0000000f) ||
	(reg32_read(MMDC_P1_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x0000000f)) {
      errorcount++; // increment the errorcount variable
      g_error_read_cal = 1; // set the g_error_read_cal
    }
  } else {
    if (reg32_read(MMDC_P0_BASE_ADDR + MPRDDLHWCTL_OFFSET) & 0x0000000f) {
      errorcount++; // increment the errorcount variable
      g_error_read_cal = 1; // set the g_error_read_cal
    }
  }
  debug( "errorcount: %d\n", errorcount );
  debug("Read calibration completed, hit enter to continue\n");
  //  getc();



  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  // Write delay Calibration
  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  debug("Starting write calibration...\n");
  // 3. Reset the read data FIFOs (two resets); only need to issue reset to PHY0 since in x64
  // mode, the reset will also go to PHY1
  // read data FIFOs reset1
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // read data FIFOs reset2
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );

  // 4. Issue the Precharge-All command to the DDR device for both chip selects
  // If only using one chip select, then precharge only the desired chip select
  if (cs0_enable == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008050); // CS0
  if (cs1_enable == 1)
    reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x04008058); // CS1

  ////////////// 5. 6. 7. set the pre-defined word ///////////////
  reg32_write((MMDC_P0_BASE_ADDR + MPPDCMPR1_OFFSET), PDDWord);
  // Issue a write access to the external DDR device by setting the bit SW_DUMMY_WR (bit 0)
  // in the MPSWDAR0 and then poll this bit until it clears to indicate completion of the
  // write access.
  reg32setbit((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET), 0);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPSWDAR_OFFSET)) & 0x00000001) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );

  // 8. Set the WR_DL_ABS_OFFSET# bits to their default values
  // Both PHYs for x64 configuration, if x32, do only PHY0
  reg32_write((MMDC_P0_BASE_ADDR + MPWRDLCTL_OFFSET), initdelay);
  if (data_bus_size == 0x2) {
    reg32_write((MMDC_P1_BASE_ADDR + MPWRDLCTL_OFFSET), initdelay);
  }
  debug( "intdel0: %08lx / intdel1: %08lx\n", 
	  reg32_read(MMDC_P0_BASE_ADDR + MPWRDLCTL_OFFSET),
	  reg32_read(MMDC_P1_BASE_ADDR + MPWRDLCTL_OFFSET));
#if 0
  reg32_write((MMDC_P0_BASE_ADDR + MPWRDLCTL_OFFSET), 0x40404040);
  if (data_bus_size == 0x2) {
    reg32_write((MMDC_P1_BASE_ADDR + MPWRDLCTL_OFFSET), 0x40404040);
  }
#endif

  // ?? this isn't in the manual. Force a measurment, for previous delay setup to effect:
  reg32_write((MMDC_P0_BASE_ADDR + MPMUR_OFFSET), 0x800);
  if (data_bus_size == 0x2) {
    reg32_write((MMDC_P1_BASE_ADDR + MPMUR_OFFSET), 0x800);
  }

  // 9. 10. Start the automatic write calibration process by asserting MPWRDLHWCTL0[HW_WR_DL_EN]
  reg32_write((MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET), 0x00000030);
  // poll for completion
  // MMDC indicates that the write data calibration had finished by setting
  // MPWRDLHWCTL[HW_WR_DL_EN] = 0
  // Also, ensure that no error bits were set
  while (reg32_read((MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET)) & 0x00000010)  {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // check both PHYs for x64 configuration, if x32, check only PHY0
  if (data_bus_size == 0x2) {
    if ((reg32_read(MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x0000000f) ||
	(reg32_read(MMDC_P1_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x0000000f)) {
      errorcount++; // increment the errorcount variable
      g_error_write_cal = 1; // set the g_error_write_cal
    }
  } else {
    if (reg32_read(MMDC_P0_BASE_ADDR + MPWRDLHWCTL_OFFSET) & 0x0000000f) {
      errorcount++; // increment the errorcount variable
      g_error_write_cal = 1; // set the g_error_write_cal
    }
  }
  debug( "errorcount: %d\n", errorcount );
  debug("Write calibration completed, hit enter to continue\n");
  //  getc();
  // Reset the read data FIFOs (two resets); only need to issue reset to PHY0 since in x64
  // mode, the reset will also go to PHY1
  // read data FIFOs reset1
  reg32_write((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P0_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );
  // read data FIFOs reset2
  reg32_write((MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET),
	      reg32_read((MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET)) | 0x80000000);
  while (reg32_read((MMDC_P1_BASE_ADDR + MPDGCTRL0_OFFSET)) & 0x80000000) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );

  // enable DDR logic power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MDPDC_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MDPDC_OFFSET)) | 0x00005500);
  // enable Adopt power down timer:
  reg32_write((MMDC_P0_BASE_ADDR + MAPSR_OFFSET),
	      reg32_read((MMDC_P0_BASE_ADDR + MAPSR_OFFSET)) & 0xfffffff7);
  //restore MDMISC value (RALAT, WALAT)
  reg32_write((MMDC_P1_BASE_ADDR + MDMISC_OFFSET), temp1);
  // device ODT back to normal:
  // reg32_write((MMDC_P0_BASE_ADDR + MPODTCTRL_OFFSET),
  // reg32_read((MMDC_P0_BASE_ADDR + MPODTCTRL_OFFSET)) & 0xfffffff7);
  //clear DQS pull ups
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS4) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS5) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS6) & 0xffff0fff);
  reg32_write(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7,
	      reg32_read(IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS7) & 0xffff0fff);
  // re-enable SDE (chip selects) if they were set initially
  if (cs1_enable_initial == 1) {
    reg32setbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 30); // set SDE_1
  }
  if (cs0_enable_initial == 1) {
    reg32setbit((MMDC_P0_BASE_ADDR + MDCTL_OFFSET), 31); // set SDE_0
  }
  // re-enable to auto refresh
  reg32_write((MMDC_P0_BASE_ADDR + MDREF_OFFSET), temp_ref);
  // clear the MDSCR (including the con_req bit)
  reg32_write((MMDC_P0_BASE_ADDR + MDSCR_OFFSET), 0x0); // CS0
  // poll to make sure the con_ack bit is clear
  while ((reg32_read(MMDC_P0_BASE_ADDR + MDSCR_OFFSET) & 0x00004000)) {
    if( withprint )
      printf( "." );
  }
 // printf( "\n" );

  // print out the registers that were updated as a result of the calibration process
  debug("MMDC registers updated from calibration \n");
  debug("\nRead DQS Gating calibration\n");
  debug("MPDGCTRL0 PHY0 (0x021b083c) = 0x%08X\n", reg32_read(0x021b083c));
  debug("MPDGCTRL1 PHY0 (0x021b0840) = 0x%08X\n", reg32_read(0x021b0840));
  debug("MPDGCTRL0 PHY1 (0x021b483c) = 0x%08X\n", reg32_read(0x021b483c));
  debug("MPDGCTRL1 PHY1 (0x021b4840) = 0x%08X\n", reg32_read(0x021b4840));
  debug("\nRead calibration\n");
  debug("MPRDDLCTL PHY0 (0x021b0848) = 0x%08X\n", reg32_read(0x021b0848));
  debug("MPRDDLCTL PHY1 (0x021b4848) = 0x%08X\n", reg32_read(0x021b4848));
  debug("\nWrite calibration\n");
  debug("MPWRDLCTL PHY0 (0x021b0850) = 0x%08X\n", reg32_read(0x021b0850));
  debug("MPWRDLCTL PHY1 (0x021b4850) = 0x%08X\n", reg32_read(0x021b4850));
  debug("\n");
  // registers below are for debugging purposes
  // these print out the upper and lower boundaries captured during read DQS gating calibration
  debug("Status registers, upper and lower bounds, for read DQS gating. \n");
  debug("MPDGHWST0 PHY0 (0x021b087c) = 0x%08X\n", reg32_read(0x021b087c));
  debug("MPDGHWST1 PHY0 (0x021b0880) = 0x%08X\n", reg32_read(0x021b0880));
  debug("MPDGHWST2 PHY0 (0x021b0884) = 0x%08X\n", reg32_read(0x021b0884));
  debug("MPDGHWST3 PHY0 (0x021b0888) = 0x%08X\n", reg32_read(0x021b0888));
  debug("MPDGHWST0 PHY1 (0x021b487c) = 0x%08X\n", reg32_read(0x021b487c));
  debug("MPDGHWST1 PHY1 (0x021b4880) = 0x%08X\n", reg32_read(0x021b4880));
  debug("MPDGHWST2 PHY1 (0x021b4884) = 0x%08X\n", reg32_read(0x021b4884));
  debug("MPDGHWST3 PHY1 (0x021b4888) = 0x%08X\n", reg32_read(0x021b4888));

  debug("errorcount: %d\n", errorcount );

  return 0;
}




U_BOOT_CMD(
	tmd,	3,	1,	do_tune_md,
	"memory display for tuning",
	"[.b, .w, .l] address [# of objects]"
);

U_BOOT_CMD(
	tmw,	4,	1,	do_tune_mw,
	"memory write (fill) for tuning",
	"[.b, .w, .l] address value [count]"
);

U_BOOT_CMD(
	tmw2,	15,	1,	do_tune_mw2,
	"memory write (fill) for tuning, ignores trailing data",
	"[.b, .w, .l] address value"
);

U_BOOT_CMD(
	tmww,	2,	1,	do_tune_mww,
	"write random data to RAM",
	"[bank]"
);

U_BOOT_CMD(
	tmrr,	2,	1,	do_tune_mrr,
	"read data and checksum from write",
	"[bank]"
);


U_BOOT_CMD(
	tmtest,	5,	1,	do_tune_mtest,
	"simple RAM read/write test",
	"[start [end [pattern [iterations]]]]"
);

U_BOOT_CMD(
	tmwcal,	11,	1,	do_tune_wcal,
	"write calibration",
	"[mr1 setting] [withprint] [wlcyc0] [wlcyc1] [wlcyc2] [wlcyc3] [wlcyc4] [wlcyc5] [wlcyc6] [wlcyc7]"
);

U_BOOT_CMD(
	tmdel,	3,	1,	do_tune_delays,
	"delay calibration",
	"[withprint] [initdelay]"
);



int do_nothing(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}

cmd_tbl_t __u_boot_cmd_question_hash Struct_Section = {
	"#",	CONFIG_SYS_MAXARGS,	1,	do_nothing,
	"comment",
	""
};

#include <common.h>
#include <i2c.h>
#include <asm/arch/mx6x_pins.h>
#include <asm/gpio.h>

#define NOVENA_SIGNATURE "Novena"
#define NOVENA_VERSION 1
#define EEPROM_ADDRESS (0xac>>1)
#define I2C_BUS 2

/*
 * For structure documentation, see:
 * http://www.kosagi.com/w/index.php?title=Novena/EEPROM
 */
struct novena_eeprom_data {
        uint8_t         signature[6];   /* 'Novena' */
        uint8_t         version;        /* 1 */
        uint8_t         reserved1;
        uint32_t        serial;
        uint8_t         mac[6];
        uint16_t        features;
} __attribute__((__packed__));

struct feature {
        uint32_t        flags;
        char            *name;
        char            *descr;
	void		(*func)(struct novena_eeprom_data *);
};

static void setup_es8328(struct novena_eeprom_data *);
static void setup_senoko(struct novena_eeprom_data *);
static void setup_retina(struct novena_eeprom_data *);
static void setup_pixelqi(struct novena_eeprom_data *);
static void setup_pcie(struct novena_eeprom_data *);
static void setup_gbit(struct novena_eeprom_data *);
static void setup_hdmi(struct novena_eeprom_data *);

struct feature features[] = {
	{
		.name  = "es8328",
		.flags = 0x01,
		.descr = "ES8328 audio codec",
		.func  = setup_es8328,
	},
	{
		.name  = "senoko",
		.flags = 0x02,
		.descr = "Power Management Board",
		.func  = setup_senoko,
	},
	{
		.name  = "retina",
		.flags = 0x04,
		.descr = "Retina-class dual-LVDS display",
		.func  = setup_retina,
	},
	{
		.name  = "pixelqi",
		.flags = 0x08,
		.descr = "PixelQi LVDS display",
		.func  = setup_pixelqi,
	},
	{
		.name  = "pcie",
		.flags = 0x10,
		.descr = "PCI Express support",
		.func  = setup_pcie,
	},
	{
		.name  = "gbit",
		.flags = 0x20,
		.descr = "Gigabit Ethernet",
		.func  = setup_gbit,
	},
	{
		.name  = "hdmi",
		.flags = 0x40,
		.descr = "HDMI Output",
		.func  = setup_hdmi,
	},
	{} /* Sentinal */
};

static void setup_backlight(void) {
	iomux_v3_cfg_t bl_pads[] = {
		/* Backlight GPIO connector */
		MX6Q_PAD_CSI0_DAT10__GPIO_5_28 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define BACKLIGHT_GPIO IMX_GPIO_NR(5, 28)
	};
	imx_iomux_v3_setup_multiple_pads(bl_pads, ARRAY_SIZE(bl_pads));
	gpio_direction_output(BACKLIGHT_GPIO, 1);
}

/*
 * Bring the audio codec out of reset.
 * While in reset, the codec pulls its I2C lines low, which hogs the bus.
 * Therefore, we want it to be out of reset as early as possible.
 */
static void setup_es8328(struct novena_eeprom_data *cfg) {
	iomux_v3_cfg_t audio_pads[] = {
		/* audio codec */
		MX6Q_PAD_DISP0_DAT23__GPIO_5_17 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define AUDIO_GPIO IMX_GPIO_NR(5, 17)
	};
	imx_iomux_v3_setup_multiple_pads(audio_pads, ARRAY_SIZE(audio_pads));
	gpio_direction_output(AUDIO_GPIO, 1);
	setenv("prep_es8328", "fdt set /soc/aips-bus@02100000/i2c@021a8000/es8328@11 status \"okay\"");
	return;
}

static void setup_senoko(struct novena_eeprom_data *cfg) {
	iomux_v3_cfg_t senoko_pads[] = {
		/* "Reprogram" pin */
		MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20 | MUX_PAD_CTRL(NO_PAD_CTRL),
		/* "Reset" pin */
		MX6Q_PAD_CSI0_VSYNC__GPIO_5_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define PMB_REPROG_GPIO IMX_GPIO_NR(5, 20)
#define PMB_RESET_GPIO IMX_GPIO_NR(5, 21)
	};
	imx_iomux_v3_setup_multiple_pads(senoko_pads, ARRAY_SIZE(senoko_pads));

	/* Reset the board */
	gpio_direction_output(PMB_RESET_GPIO, 0);

	/* Disable the "reprogram" signal prior to booting board */
	gpio_direction_output(PMB_REPROG_GPIO, 0);

	/* Bring the board out of reset */
	udelay(10000);
	gpio_direction_output(PMB_RESET_GPIO, 1);

	setenv("prep_senoko", "true");

	return;
}

static void setup_retina(struct novena_eeprom_data *cfg) {
	setup_backlight();
	setenv("prep_retina",
	"fdt set "
	"/soc/aips-bus@02100000/i2c@021a8000/stdp4028@73 status \"okay\"; "
	"fdt set "
	"/soc/aips-bus@02000000/ldb@020e0008 status \"okay\"; "
	"fdt set "
	"/soc/aips-bus@02000000/ldb@020e0008/lvds-channel@0 status \"okay\"; "
	"fdt set "
	"/soc/aips-bus@02000000/ldb@020e0008/lvds-channel@1 status \"okay\"");
	return;
}

static void setup_pixelqi(struct novena_eeprom_data *cfg) {
	setup_backlight();
	/* Remainder not implemented */
	setenv("prep_pixelqi", "true");
	return;
}

static void setup_pcie(struct novena_eeprom_data *cfg) {
	iomux_v3_cfg_t pcie_pads[] = {
		/* "Reset" pin */
		MX6Q_PAD_EIM_D29__GPIO_3_29 | MUX_PAD_CTRL(NO_PAD_CTRL),
		/* "Power on" pin */
		MX6Q_PAD_GPIO_17__GPIO_7_12 | MUX_PAD_CTRL(NO_PAD_CTRL),
		/* "Wake up" pin (input) */
		MX6Q_PAD_EIM_D22__GPIO_3_22 | MUX_PAD_CTRL(NO_PAD_CTRL),
		/* "Disable endpoint" (rfkill) pin */
		MX6Q_PAD_EIM_A22__GPIO_2_16 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define PCIE_RESET_GPIO IMX_GPIO_NR(3, 29)
#define PCIE_POWER_ON_GPIO IMX_GPIO_NR(7, 12)
#define PCIE_WAKE_UP_GPIO IMX_GPIO_NR(3, 22)
#define PCIE_DISABLE_GPIO IMX_GPIO_NR(2, 16)
	};
	imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));

	/* Ensure PCIe is powered down */
	gpio_direction_output(PCIE_POWER_ON_GPIO, 0);

	/* Put the card into reset */
	gpio_direction_output(PCIE_RESET_GPIO, 0);

	/* Input signal to wake system from mPCIe card */
	gpio_direction_input(PCIE_WAKE_UP_GPIO);

	/* Drive RFKILL high, to ensure the radio is turned on */
	gpio_direction_output(PCIE_DISABLE_GPIO, 1);

	/* Now power PCIe back up */
	gpio_direction_output(PCIE_POWER_ON_GPIO, 1);

	setenv("prep_pcie", "fdt set /soc/pcie@0x01000000 status \"okay\"");

	return;
}

static void setup_gbit(struct novena_eeprom_data *cfg) {
	int random_mac = 0;
	if (cfg->mac[0] == 0xff && cfg->mac[1] == 0xff
	 && cfg->mac[2] == 0xff && cfg->mac[3] == 0xff
	 && cfg->mac[4] == 0xff && cfg->mac[5] == 0xff)
		random_mac = 1;
	if (cfg->mac[0] == 0x00 && cfg->mac[1] == 0x00
	 && cfg->mac[2] == 0x00 && cfg->mac[3] == 0x00
	 && cfg->mac[4] == 0x00 && cfg->mac[5] == 0x00)
		random_mac = 1;

	if (random_mac)
		setenv("prep_gbit", "fdt rm /soc/aips-bus@02100000/ethernet@02188000 mac-address; fdt set /soc/aips-bus@02100000/ethernet@02188000 status \"okay\"");
	else {
		char e[512];
		sprintf(e, "fdt set /soc/aips-bus@02100000/ethernet@02188000 mac-address \"[%02x %02x %02x %02x %02x %02x]\"; fdt set /soc/aips-bus@02100000/ethernet@02188000 status \"okay\"",
				cfg->mac[0], cfg->mac[1],
				cfg->mac[2], cfg->mac[3],
				cfg->mac[4], cfg->mac[5]);
		setenv("prep_gbit", e);
	}
}

static void setup_hdmi(struct novena_eeprom_data *cfg) {
	iomux_v3_cfg_t hdmi_pads[] = {
		/* "Ghost HPD" pin */
		MX6Q_PAD_EIM_A24__GPIO_5_4 | MUX_PAD_CTRL(NO_PAD_CTRL),
#define HDMI_GHOST_HPD IMX_GPIO_NR(5, 4)
	};
	imx_iomux_v3_setup_multiple_pads(hdmi_pads, ARRAY_SIZE(hdmi_pads));

	/* Reset the board */
	gpio_direction_input(HDMI_GHOST_HPD);

	setenv("prep_hdmi",
		"fdt set /soc/aips-bus@02000000/hdmi@0120000 status \"okay\"");
	return;
}

int setup_peripherals(void)
{
	struct novena_eeprom_data cfg;
	struct feature *feature;


	i2c_set_bus_num(I2C_BUS);
	i2c_init(100000, 0);
	if (i2c_read(EEPROM_ADDRESS, 0, 2, (void *)&cfg, sizeof(cfg)) != 0) {
		/*
		 * HACK: The ES8328 hogs the I2C bus.  Without turning on the
		 * ES8328, we're unable to query the personality EEPROM.
		 * Assume it exists, and try again.
		 */
		setup_es8328(NULL);

		/* Let the ES8328 come up */
		udelay(10000);

		if (i2c_read(EEPROM_ADDRESS, 0, 2,
				(void *)&cfg, sizeof(cfg)) != 0) {
			printf("Error: Unable to read personality EEPROM\n");
			return 1;
		}
	}

	if (strncmp((void *)cfg.signature,
			(void *)NOVENA_SIGNATURE,
			sizeof(cfg.signature))) {
		printf("Warning: Personality EEPROM signature not valid. "
			"Using built-in defaults.\n");
		return 2;
	}

	if (cfg.version != NOVENA_VERSION) {
		printf("Warning: Personality EEPROM version %d not supported\n",
				cfg.version);
		return 3;
	}

	feature = features;
	while(feature->name) {
		if (cfg.features & feature->flags) {
			printf("    Peripheral: Enabling %s (%s)\n",
					feature->name, feature->descr);
			feature->func(&cfg);
		}
		feature++;
	}

	return 0;
}

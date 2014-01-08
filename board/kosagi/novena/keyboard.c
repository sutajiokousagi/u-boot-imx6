#include <common.h>
#include <asm/gpio.h>
#include <asm/arch/mx6x_pins.h>

struct button_key {
	char const	*name;
	unsigned	gpnum;
	char		ident;
};

static struct button_key const buttons[] = {
        {"debug",       IMX_GPIO_NR(4, 14),     'D'},
};

#define BUTTON_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	\
        PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |	\
        PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

/* Button assignments for J14 */
static iomux_v3_cfg_t button_pads[] = {
	/* Debug */
	MX6Q_PAD_KEY_COL4__GPIO_4_14	| MUX_PAD_CTRL(BUTTON_PAD_CTRL),
};

void setup_buttons(void)
{
	imx_iomux_v3_setup_multiple_pads(button_pads, ARRAY_SIZE(button_pads));
}

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
        setenv("readkeys", envvalue);
        return numpressed == 0;
}

U_BOOT_CMD(
        kbd, 1, 1, do_kbd,
        "Tests for keypresses, sets 'readkeys' environment variable",
        "Returns 0 (true) to shell if key is pressed."
);

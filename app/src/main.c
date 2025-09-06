#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#define SPI_BUS DT_NODELABEL(spi0)
#define SPI_OP SPI_OP_MODE_MASTER | SPI_WORD_SET(8)
#define DAC_LDAC DT_NODELABEL(dac_ldac)

#define AMUX_EN DT_NODELABEL(amux_en)
#define AMUX_A0 DT_NODELABEL(amux_a0)
#define AMUX_A1 DT_NODELABEL(amux_a1)

LOG_MODULE_REGISTER(main);

void write_dac_channel(const struct device *spi_dev, int channel, float voltage, float vref)
{
	// Validate channel value
	if (channel < 0 || channel > 1)
	{
		LOG_ERR("Invalid DAC channel: %d", channel);
		return;
	}

	// Calculate DAC code based on voltage and reference
	uint16_t code = (uint16_t)((voltage / vref) * 4096.0f);

	// Build command: shift channel into position and split code into two bytes
	uint8_t addr = channel << 3;
    uint8_t cmd[3] = { addr, (uint8_t)((code >> 8) & 0x0F), (uint8_t)(code & 0xFF) };

    struct spi_buf tx_buf = { .buf = cmd, .len = sizeof(cmd) };
    struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

    const struct spi_config spi_cfg = {
        .frequency = DT_PROP(SPI_BUS, clock_frequency),
        .operation = SPI_OP,
        .cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(mcp48), 0U),
    };

    int ret = spi_write(spi_dev, &spi_cfg, &tx_bufs);
    if (ret) {
        LOG_ERR("spi_write failed: %d", ret);
    }
}

void read_adc(const struct device *spi_dev, float *voltage, float vref)
{

	uint8_t rx_buf[2] = {0};
	struct spi_buf rx_bufs = {.buf = rx_buf, .len = sizeof(rx_buf)};
	struct spi_buf_set rx_set = {.buffers = &rx_bufs, .count = 1};

	const struct spi_config spi_cfg = {
		.frequency = DT_PROP(SPI_BUS, clock_frequency),
		.operation = SPI_OP,
		.cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(mcp3201), 0U),
	};

	int ret = spi_read(spi_dev, &spi_cfg, &rx_set);
	if (ret)
	{
		LOG_ERR("spi_read failed: %d", ret);
		return;
	}

	// Process received ADC value
	uint16_t adc_code = rx_buf[0] << 8 | rx_buf[1];
	adc_code = adc_code; // 12-bit ADC
	*voltage = ((float)adc_code / 4096.0f) * vref;

	return;
}

float voltage = 0.0f;

static int handle_shell_write_dac(const struct shell *shell, size_t argc, char **argv)
{

	if (argc != 3)
	{
		shell_print(shell, "Usage: dac write <channel> <voltage>");
		return -1;
	}

	int channel = atoi(argv[1]);
	LOG_INF("Channel arg: %s", argv[1]);
	double voltage = atof(argv[2]);
	LOG_INF("Voltage arg: %s", argv[2]);

	LOG_INF("Channel: %d, Voltage: %.3f", channel, voltage);

	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	write_dac_channel(spi_dev, channel, (float)voltage, 3.3f);

	// ldac pulse
	struct gpio_dt_spec dac_ldac = GPIO_DT_SPEC_GET(DAC_LDAC, gpios);
	gpio_pin_set_dt(&dac_ldac, 1);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 0);

}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dac,
	SHELL_CMD(write, NULL, "<channel> <voltage>", handle_shell_write_dac),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dac, &sub_dac, "DAC commands", NULL);

// Added ADC shell command to read ADC value from the single channel
static int handle_shell_read_adc(const struct shell *shell, size_t argc, char **argv)
{
	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	float adc_voltage = 0.0f;
	read_adc(spi_dev, &adc_voltage, 3.3f);
	shell_print(shell, "ADC Voltage: %.3f V", adc_voltage);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc,
	SHELL_CMD(read, NULL, "Read ADC value", handle_shell_read_adc),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(adc, &sub_adc, "ADC commands", NULL);

// Replacing the old 'amux set' command with new 'amux mode' command
static int handle_shell_set_amux_mode(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_print(shell, "Usage: amux mode <N> where N = 0,1, or 2");
		return -1;
	}

	int mode = atoi(argv[1]);
	if (mode < 0 || mode > 2) {
		shell_print(shell, "Error: mode must be 0, 1, or 2");
		return -1;
	}

	int a0, a1;
	const char *resistor;
	switch (mode) {
		case 0:
			a0 = 0; a1 = 0; resistor = "10M"; break;
		case 1:
			a0 = 0; a1 = 1; resistor = "100k"; break;
		case 2:
			a0 = 1; a1 = 0; resistor = "1k"; break;
		default:
			return -1; // Should not reach here
	}

	struct gpio_dt_spec amuxa0 = GPIO_DT_SPEC_GET(AMUX_A0, gpios);
	struct gpio_dt_spec amuxa1 = GPIO_DT_SPEC_GET(AMUX_A1, gpios);
	gpio_pin_set_dt(&amuxa0, a0);
	gpio_pin_set_dt(&amuxa1, a1);

	shell_print(shell, "Analog multiplexer set to mode %d: A0=%d, A1=%d, resistor: %s", mode, a0, a1, resistor);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_amux,
	SHELL_CMD(mode, NULL, "Set analog multiplexer mode (0-2)", handle_shell_set_amux_mode),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(amux, &sub_amux, "Analog multiplexer commands", NULL);

int main()
{

	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	struct gpio_dt_spec dac_ldac = GPIO_DT_SPEC_GET(DAC_LDAC, gpios);
	gpio_pin_configure_dt(&dac_ldac, GPIO_OUTPUT_ACTIVE);

	struct gpio_dt_spec amuxen = GPIO_DT_SPEC_GET(AMUX_EN, gpios);
	struct gpio_dt_spec amuxa0 = GPIO_DT_SPEC_GET(AMUX_A0, gpios);
	struct gpio_dt_spec amuxa1 = GPIO_DT_SPEC_GET(AMUX_A1, gpios);

	gpio_pin_configure_dt(&amuxen, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure_dt(&amuxa0, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&amuxa1, GPIO_OUTPUT_INACTIVE);

	// write_dac_command(spi_dev, 0, (uint8_t[]){0x08<<3, 0x00, 0b1010});
	// write_dac_command(spi_dev, 0, (uint8_t[]){0x0A<<3, 0b0000, 0b0});
	// write_dac_command(spi_dev, 0, 0x08, 0b1010);
	// write_dac_command(spi_dev, 0, 0x0A, 0b1010 << 8 | 0b00 < 7);

	write_dac_channel(spi_dev, 0, 0.5f, 3.3f);
	write_dac_channel(spi_dev, 1, 0.5f, 3.3f);

	gpio_pin_set_dt(&dac_ldac, 1);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 0);

	gpio_pin_set_dt(&amuxen, 1);
	gpio_pin_set_dt(&amuxa0, 1);
	gpio_pin_set_dt(&amuxa1, 0);

	while (1)
	{

		// gpio_pin_set_dt(&dac_ldac, 1);
		// k_msleep(10);
		// gpio_pin_set_dt(&dac_ldac, 0);

		// read_adc(spi_dev, &voltage, 2.5f);

		// printk("ADC Voltage: %.3f V\r\n", voltage);
		k_sleep(K_MSEC(333));
	}
	return 0;
}
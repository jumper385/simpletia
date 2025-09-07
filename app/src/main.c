#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#define SPI_BUS DT_NODELABEL(spi0)
#define SPI_OP SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB 
#define DAC_LDAC DT_NODELABEL(dac_ldac)

#define AMUX_EN DT_NODELABEL(amux_en)
#define AMUX_A0 DT_NODELABEL(amux_a0)
#define AMUX_A1 DT_NODELABEL(amux_a1)

LOG_MODULE_REGISTER(main);

static float current_dac[2] = {0.0f, 0.0f};

void write_dac_channel(const struct device *spi_dev, int channel, uint16_t code, float vref)
{
	// Validate channel value
	if (channel < 0 || channel > 1)
	{
		LOG_ERR("Invalid DAC channel: %d", channel);
		return;
	}

	// Build command: shift channel into position and split code into two bytes
	uint8_t addr = channel << 3 | 0b00 << 1;
    // uint8_t cmd[3] = { addr, (uint8_t)((code >> 8) & 0x0F), (uint8_t)(code & 0xF) };
	uint8_t cmd[3] = { addr, code >> 8 & 0b00001111, code & 0xFF};

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
	adc_code = adc_code;
	// print out hex code 0xXXX
	LOG_INF("ADC Code: 0x%03X", adc_code);

	*voltage = ((float)adc_code / 8192.0f) * vref;

	return;
}

float voltage = 0.0f;

static int handle_shell_write_dac(const struct shell *shell, size_t argc, char **argv)
{

	if (argc != 3)
	{
		shell_print(shell, "Usage: dac write <channel> <binary_code>");
		shell_print(shell, "       channel: 0 or 1");
		shell_print(shell, "       binary_code: 12-bit binary value (e.g., 101010101010)");
		return -1;
	}

	int channel = atoi(argv[1]);
	
	// Parse the 12-bit binary string
	const char *binary_str = argv[2];
	size_t len = strlen(binary_str);
	if (len != 12) {
		shell_print(shell, "Error: Binary code must be exactly 12 bits (e.g., 101010101010)");
		return -1;
	}
	
	uint16_t code = 0;
	for (int i = 0; i < 12; i++) {
		if (binary_str[i] != '0' && binary_str[i] != '1') {
			shell_print(shell, "Error: Binary code must contain only 0s and 1s");
			return -1;
		}
		code = (code << 1) | (binary_str[i] - '0');
	}

	code = 0b111111111111 & code; // Ensure code is 12-bit
	
	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	write_dac_channel(spi_dev, channel, code, 3.3f);
	
	// Calculate voltage from code for display purposes
	float voltage = ((float)code / 4096.0f) * 3.3f;

	// ldac pulse
	struct gpio_dt_spec dac_ldac = GPIO_DT_SPEC_GET(DAC_LDAC, gpios);
	gpio_pin_set_dt(&dac_ldac, 0);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 1);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 0);

	// Update the global DAC configuration and print out the current settings
	if (channel >= 0 && channel < 2) {
		current_dac[channel] = voltage;
	}
	shell_print(shell, "DAC Channel %d set with binary: %s (code: %u, ~%.3f V)", 
		channel, binary_str, code, voltage);
	shell_print(shell, "Current DAC configuration: Channel 0: %.3f V, Channel 1: %.3f V", 
		current_dac[0], current_dac[1]);

	return 0;
}

// Added DAC read command to display current DAC configuration
static int handle_shell_read_dac(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Current DAC configuration: Channel 0: %.3f V, Channel 1: %.3f V", current_dac[0], current_dac[1]);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dac,
	SHELL_CMD(write, NULL, "Set DAC: <channel> <binary_code>", handle_shell_write_dac),
	SHELL_CMD(read, NULL, "Show current DAC configuration", handle_shell_read_dac),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dac, &sub_dac, "DAC commands", NULL);

// Added ADC shell command to read ADC value from the single channel
static int handle_shell_read_adc(const struct shell *shell, size_t argc, char **argv)
{
	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	float adc_voltage = 0.0f;
	read_adc(spi_dev, &adc_voltage, 2.5f);
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
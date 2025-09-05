#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>

#define SPI_BUS DT_NODELABEL(spi0)
#define SPI_OP  SPI_OP_MODE_MASTER |SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE
#define DAC_LDAC DT_NODELABEL(dac_ldac)

LOG_MODULE_REGISTER(main);

void write_dac_channel(const struct device *spi_dev, int channel, float voltage, float vref)
{
    // Validate channel value
    if (channel < 0 || channel > 1) {
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

void read_adc(const struct device *spi_dev, float *voltage, float vref) {

	uint8_t rx_buf[2] = {0};
	struct spi_buf rx_bufs = { .buf = rx_buf, .len = sizeof(rx_buf) };
	struct spi_buf_set rx_set = { .buffers = &rx_bufs, .count = 1 };

	const struct spi_config spi_cfg = {
		.frequency = DT_PROP(SPI_BUS, clock_frequency),
		.operation = SPI_OP,
		.cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(mcp3201), 0U),
	};

	int ret = spi_read(spi_dev, &spi_cfg, &rx_set);
	if (ret) {
		LOG_ERR("spi_read failed: %d", ret);
		return;
	}

	// Process received ADC value
	uint16_t adc_code = rx_buf[0] << 8 | rx_buf[1];
	LOG_INF("ADC Raw: %d", adc_code);
	*voltage = ((float)adc_code / 8192.0f) * vref;

	return;
}

float voltage = 0.0f;

int main() {

	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	struct gpio_dt_spec dac_ldac = GPIO_DT_SPEC_GET(DAC_LDAC, gpios);
    gpio_pin_configure_dt(&dac_ldac, GPIO_OUTPUT_ACTIVE);

	// write_dac_channel(spi_dev, 0, 1.1f, 3.3f);
	write_dac_channel(spi_dev, 1, 1.25f, 3.3f);

	gpio_pin_set_dt(&dac_ldac, 1);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 0);

	while (1) {

		read_adc(spi_dev, &voltage, 2.5f);

		LOG_INF("HELLO WORLD");
		LOG_INF("ADC Voltage: %.2f V", voltage);	
		k_sleep(K_MSEC(100));
	}
	return 0;
}
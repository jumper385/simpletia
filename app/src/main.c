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
static float voltage_mode_offsets[3] = {0.0f, 0.0f, 0.0f}; // offsets for modes 0, 1, 2
static int amux_mode = 0; 
static bool continuous_sampling = false;
static K_THREAD_STACK_DEFINE(sampling_stack, 1024);
static struct k_thread sampling_thread;
static k_tid_t sampling_tid; 

void write_dac_channel(const struct device *spi_dev, int channel, float voltage, float vref)
{
	// Validate channel value
	if (channel < 0 || channel > 1)
	{
		LOG_ERR("Invalid DAC channel: %d", channel);
		return;
	}

	// Calculate DAC code based on voltage and reference (10-bit resolution)
	uint16_t code = (uint16_t)((voltage / vref) * 1023.0f);
	
	// Build command: shift channel into position and split code into two bytes
	uint8_t addr = channel << 3 | 0b00 << 1;
	uint8_t cmd[3] = { addr, (uint8_t)((code >> 8) & 0x03), (uint8_t)(code & 0xFF) };

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
	// LOG_INF("ADC Code: 0x%03X", adc_code);

	*voltage = ((float)adc_code / 8192.0f) * vref;

	// LOG_INF("ADC Voltage: %.3f V", *voltage);

	return;
}

float voltage = 0.0f;

static int handle_shell_write_dac(const struct shell *shell, size_t argc, char **argv)
{

	if (argc != 3)
	{
		shell_print(shell, "Usage: dac write <channel> <voltage>");
		shell_print(shell, "       channel: 0 or 1");
		shell_print(shell, "       voltage: voltage value in range 0-3.3V");
		return -1;
	}

	int channel = atoi(argv[1]);
	
	// Parse the voltage value
	double voltage = atof(argv[2]);
	if (voltage < 0.0 || voltage > 3.3) {
		shell_print(shell, "Warning: Voltage %.3f V is outside the recommended range (0-3.3V)", voltage);
	}
	
	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	write_dac_channel(spi_dev, channel, (float)voltage, 3.3f);
	
	// Calculate code for display purposes (10-bit DAC)
	uint16_t code = (uint16_t)((voltage / 3.3f) * 1023.0f);

	// ldac pulse
	struct gpio_dt_spec dac_ldac = GPIO_DT_SPEC_GET(DAC_LDAC, gpios);
	gpio_pin_set_dt(&dac_ldac, 0);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 1);
	k_msleep(10);
	gpio_pin_set_dt(&dac_ldac, 0);

	// Update the global DAC configuration and print out the current settings
	if (channel >= 0 && channel < 2) {
		current_dac[channel] = (float)voltage;
	}
	shell_print(shell, "DAC Channel %d set to %.3f V (10-bit code: %u)", 
		channel, voltage, code);
	shell_print(shell, "Current DAC configuration: Channel 0: %.3f V, Channel 1: %.3f V", 
		current_dac[0], current_dac[1]);

	return 0;
}

static int handle_shell_read_dac(const struct shell *shell, size_t argc, char **argv)
{
	shell_print(shell, "Current DAC configuration: Channel 0: %.3f V, Channel 1: %.3f V", current_dac[0], current_dac[1]);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dac,
	SHELL_CMD(write, NULL, "Set DAC: <channel> <voltage>", handle_shell_write_dac),
	SHELL_CMD(read, NULL, "Show current DAC configuration", handle_shell_read_dac),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(dac, &sub_dac, "DAC commands", NULL);

static int handle_shell_read_adc(const struct shell *shell, size_t argc, char **argv)
{
	struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
	float adc_voltage = 0.0f;
	read_adc(spi_dev, &adc_voltage, 2.520f);

	// calculate current	
	float current_reading;
	float scale_factor;
	char *unit;
	switch (amux_mode) {
		case 0:
			current_reading = -1 * (adc_voltage - current_dac[1]) / 10e6; // 10M resistor
			unit = "pA";
			scale_factor = 1e12;
			break;
		case 1:
			current_reading = -1 * (adc_voltage - current_dac[1]) / 146e3; // 100k resistor
			unit = "nA";
			scale_factor = 1e9;
			break;
		case 2:
			current_reading = -1 * (adc_voltage - current_dac[1]) / 1e3; // 1k resistor
			unit = "uA";
			scale_factor = 1e6;
			break;
		default:
			current_reading = 0.0f; // Should not reach here
			unit = "A";
			scale_factor = 1.0f;
			break;
	}

	shell_print(shell, "Current: %.3f %s", current_reading * scale_factor, unit);
	return 0;
}

// Thread function for continuous ADC sampling
void continuous_sampling_thread(void *arg1, void *arg2, void *arg3)
{
    struct device *spi_dev = DEVICE_DT_GET(SPI_BUS);
    float adc_voltage;
    int sample_count = 0;
    
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    while (continuous_sampling) {
        read_adc(spi_dev, &adc_voltage, 2.520f);
        
        // Calculate current based on the AMUX mode
        float current_reading;
        float scale_factor;
        char *unit;
        switch (amux_mode) {
            case 0:
                current_reading = -1 * (adc_voltage - voltage_mode_offsets[0] - current_dac[1]) / 10e6;
                unit = "pA";
                scale_factor = 1e12;
                break;
            case 1:
                current_reading = -1 * (adc_voltage - voltage_mode_offsets[1] - current_dac[1]) / 146e3;
                unit = "nA";
                scale_factor = 1e9;
                break;
            case 2:
                current_reading = -1 * (adc_voltage - voltage_mode_offsets[2] - current_dac[1]) / 1e3;
                unit = "uA";
                scale_factor = 1e6;
                break;
            default:
                current_reading = 0.0f;
                unit = "A";
                scale_factor = 1.0f;
                break;
        }
        
        printk("Sample %d: Current = %.3f %s\n", sample_count++, current_reading * scale_factor, unit);
        k_msleep(100); // Sample every 100ms
    }
}

// Command to control continuous sampling mode
static int handle_shell_adc_continuous(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_print(shell, "Usage: adc continuous <on|off>");
        return -1;
    }
    
    if (strcmp(argv[1], "on") == 0) {
        if (continuous_sampling) {
            shell_print(shell, "Continuous sampling is already running");
            return 0;
        }
        
        continuous_sampling = true;
        shell_print(shell, "Starting continuous ADC sampling...");
        
        // Start sampling thread
        sampling_tid = k_thread_create(&sampling_thread, sampling_stack,
                                     K_THREAD_STACK_SIZEOF(sampling_stack),
                                     continuous_sampling_thread,
                                     NULL, NULL, NULL,
                                     5, 0, K_NO_WAIT);
        
        k_thread_name_set(sampling_tid, "adc_sampling");
        
    } else if (strcmp(argv[1], "off") == 0) {
        if (!continuous_sampling) {
            shell_print(shell, "Continuous sampling is not running");
            return 0;
        }
        
        continuous_sampling = false;
        shell_print(shell, "Stopping continuous ADC sampling...");
        
        // Thread will terminate itself when continuous_sampling becomes false
        k_thread_join(&sampling_thread, K_SECONDS(1));
        
    } else {
        shell_print(shell, "Invalid argument: %s (must be 'on' or 'off')", argv[1]);
        return -1;
    }
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_adc,
	SHELL_CMD(read, NULL, "Read ADC value", handle_shell_read_adc),
    SHELL_CMD(continuous, NULL, "Start/stop continuous sampling: <on|off>", handle_shell_adc_continuous),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(adc, &sub_adc, "ADC commands", NULL);

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

	amux_mode = mode;

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

// Helper function to ensure continuous sampling is off when exiting the application
static void app_exit_handler(void)
{
    if (continuous_sampling) {
        continuous_sampling = false;
        k_thread_join(&sampling_thread, K_SECONDS(1));
        printk("Stopped continuous sampling on exit\n");
    }
}

int main()
{
	// Register exit handler to clean up resources
	atexit(app_exit_handler);

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

	// default configs
	write_dac_channel(spi_dev, 0, 0.0f, 3.3f);
	write_dac_channel(spi_dev, 1, 1.25f, 3.3f);
	current_dac[0] = 0.0f;
	current_dac[1] = 1.25f;
	amux_mode = 2;

	while (1)
	{
		k_sleep(K_MSEC(333));
	}
	return 0;
}
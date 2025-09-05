#include <zephyr/kernel.h>

int counter = 0;

int main() {
	while (1) {

		counter ++;

		printk("HELLO WORLD\r\n");
		k_sleep(K_MSEC(1000));
	}
	return 0;
}
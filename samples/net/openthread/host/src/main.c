#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/openthread.h>
#include <openthread/thread.h>

LOG_MODULE_REGISTER(coap_client_utils, CONFIG_NET_UTILS_LOG_LEVEL);

/* 1000 msec = 1 sec */
#define BLINK_SLEEP_TIME_MS   1000
#define OT_SLEEP_TIME_MS      2000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* global variable to store led status */
static bool led_state = true;

/* static work queue callbacks*/
static void blink_led(void){
    for(;;){
        gpio_pin_toggle_dt(&led);
        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");
        k_msleep(BLINK_SLEEP_TIME_MS);
    }   
}

static void init_open_thread(void){
    int error = 0;
    k_msleep(10000);
    error = openthread_start(openthread_get_default_context());
    for(;;){
        //error = !error;
        printf("Open thread state: %s\n", error!=0 ? "Fail in init" : "Correctly init");
        k_msleep(OT_SLEEP_TIME_MS);
    }
    
}

K_THREAD_DEFINE(blink_tid, 1024, blink_led, NULL, NULL, NULL, 7, 0, 0);
//K_THREAD_DEFINE(open_thread_tid, 4096, init_open_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
    int error = 0;
	
    if (!gpio_is_ready_dt(&led)) {
	    return 0;
	}

	error = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (error < 0) {
	    return 0;
	}

    k_thread_start(blink_tid);
    printf("Start blink led example");

    //k_thread_start(open_thread_tid);
    printf("Start open thread");
}

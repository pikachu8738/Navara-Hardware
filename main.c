#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <string.h>

#define MULTI_PRESS_WINDOW_MS 2000
#define DEBOUNCE_DELAY_MS 150

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec sim_pwr = GPIO_DT_SPEC_GET(DT_ALIAS(sim_pwr), gpios);
static const struct pwm_dt_spec haptic = PWM_DT_SPEC_GET(DT_ALIAS(haptic));
static const struct pwm_dt_spec led_red = PWM_DT_SPEC_GET(DT_ALIAS(led_red));
static const struct device *sim_uart = DEVICE_DT_GET(DT_ALIAS(sim_uart));

static struct gpio_callback button_cb;
static struct k_work_delayable eval_work;

static volatile int press_count = 0;
static volatile int64_t last_press = 0;

void trigger_haptic(int pulses) {
    for(int i = 0; i < pulses; i++) {
        pwm_set_dt(&haptic, PWM_MSEC(20), PWM_MSEC(10));
        k_msleep(250);
        pwm_set_dt(&haptic, PWM_MSEC(20), 0);
        k_msleep(150);
    }
}

void send_cmd(const char *cmd) {
    for (int i = 0; i < strlen(cmd); i++) uart_poll_out(sim_uart, cmd[i]);
    uart_poll_out(sim_uart, '');
    uart_poll_out(sim_uart, '
');
    k_msleep(100); 
}

void dispatch_alert(bool full_sos) {
    gpio_pin_set_dt(&sim_pwr, 1);
    k_msleep(500);
    gpio_pin_set_dt(&sim_pwr, 0);
    k_msleep(3000);
    
    send_cmd("AT+CGPS=1");
    k_msleep(2000);
    send_cmd("AT+CGPSINFO");
    
    if (full_sos) {
        send_cmd("AT+CMGF=1");
        send_cmd("AT+CMGS="+910000000000""); 
        send_cmd("SOS ALERT TRIGGERED.");
        uart_poll_out(sim_uart, 26); 
    } else {
        send_cmd("AT+HTTPINIT");
    }
}

static void eval_handler(struct k_work *work) {
    int count = press_count;
    press_count = 0; 

    if (count >= 3) {
        trigger_haptic(3);
        pwm_set_dt(&led_red, PWM_MSEC(20), PWM_MSEC(20));
        bt_enable(NULL);
        dispatch_alert(true);
    } else if (count > 0) {
        trigger_haptic(1);
        bt_enable(NULL);
        dispatch_alert(false);
    }
    
    k_msleep(2000);
    pwm_set_dt(&led_red, PWM_MSEC(20), 0);
}

void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int64_t now = k_uptime_get();
    if (now - last_press > DEBOUNCE_DELAY_MS) {
        press_count++;
        last_press = now;
        k_work_reschedule(&eval_work, K_MSEC(MULTI_PRESS_WINDOW_MS));
    }
}

int main(void) {
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_configure_dt(&sim_pwr, GPIO_OUTPUT_INACTIVE);
    
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&button_cb, button_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb);

    k_work_init_delayable(&eval_work, eval_handler);

    while (1) k_sleep(K_FOREVER);
    return 0;
}

// Copyright (C) 2023 Joey Troy
// All rights reserved.
//
// You can use this software under the terms of 'INDIGO Astronomy
// open-source license' (see LICENSE.md).
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHORS 'AS IS' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// version history
// 1.0 by Joey Troy  - Code modified from Rumen G. Bogdanovski
// 2.0 by Seven Watt - Use pigpio library for GPIO and PWM control. 
//                     Use software PWM on all pins.
//                     Use heater property to enable Alpaca PWM control

/** INDIGO ZWO Power Ports ASIAIR AUX driver
 \file indigo_aux_asiair.c
 */

#include "indigo_aux_asiair.h"

#define DRIVER_VERSION         0x0003
#define AUX_ASIAIR_NAME     "ZWO Power Ports ASIAIR v2"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>


#include <pigpiod_if2.h>

#include <indigo/indigo_driver_xml.h>
#include <indigo/indigo_io.h>
#include <indigo/indigo_client.h>
#include <indigo/indigo_dome_driver.h>
#include <indigo/indigo_aux_driver.h>

#define PRIVATE_DATA                      ((asiair_private_data *)device->private_data)

#define AUX_RELAYS_GROUP	"Power Pin Control"

#define AUX_OUTLET_NAMES_PROPERTY      (PRIVATE_DATA->outlet_names_property)
#define AUX_OUTLET_NAME_1_ITEM         (AUX_OUTLET_NAMES_PROPERTY->items + 0)
#define AUX_OUTLET_NAME_2_ITEM         (AUX_OUTLET_NAMES_PROPERTY->items + 1)
#define AUX_OUTLET_NAME_3_ITEM         (AUX_OUTLET_NAMES_PROPERTY->items + 2)
#define AUX_OUTLET_NAME_4_ITEM         (AUX_OUTLET_NAMES_PROPERTY->items + 3)

#define AUX_OUTLET_TYPES_PROPERTY_NAME "AUX_OUTLET_TYPES"
#define AUX_OUTLET_TYPE_1_ITEM_NAME "POWER_OUTLET_TYPE_1"
#define AUX_OUTLET_TYPE_2_ITEM_NAME "POWER_OUTLET_TYPE_2"
#define AUX_OUTLET_TYPE_3_ITEM_NAME "POWER_OUTLET_TYPE_3"
#define AUX_OUTLET_TYPE_4_ITEM_NAME "POWER_OUTLET_TYPE_4"

#define AUX_OUTLET_TYPES_PROPERTY (PRIVATE_DATA->outlet_types_property)
#define AUX_OUTLET_TYPE_1_ITEM (AUX_OUTLET_TYPES_PROPERTY->items + 0)
#define AUX_OUTLET_TYPE_2_ITEM (AUX_OUTLET_TYPES_PROPERTY->items + 1)
#define AUX_OUTLET_TYPE_3_ITEM (AUX_OUTLET_TYPES_PROPERTY->items + 2)
#define AUX_OUTLET_TYPE_4_ITEM (AUX_OUTLET_TYPES_PROPERTY->items + 3)

#define AUX_GPIO_OUTLET_PROPERTY      (PRIVATE_DATA->gpio_outlet_property)
#define AUX_GPIO_OUTLET_1_ITEM        (AUX_GPIO_OUTLET_PROPERTY->items + 0)
#define AUX_GPIO_OUTLET_2_ITEM        (AUX_GPIO_OUTLET_PROPERTY->items + 1)
#define AUX_GPIO_OUTLET_3_ITEM        (AUX_GPIO_OUTLET_PROPERTY->items + 2)
#define AUX_GPIO_OUTLET_4_ITEM        (AUX_GPIO_OUTLET_PROPERTY->items + 3)

#define AUX_OUTLET_PULSE_LENGTHS_PROPERTY      (PRIVATE_DATA->gpio_outlet_pulse_property)
#define AUX_OUTLET_PULSE_LENGTHS_1_ITEM        (AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + 0)
#define AUX_OUTLET_PULSE_LENGTHS_2_ITEM        (AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + 1)
#define AUX_OUTLET_PULSE_LENGTHS_3_ITEM        (AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + 2)
#define AUX_OUTLET_PULSE_LENGTHS_4_ITEM        (AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + 3)

#define AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY         (PRIVATE_DATA->gpio_outlet_frequencies_property)
#define AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_1_ITEM (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + 0)
#define AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_2_ITEM (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + 1)
#define AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_3_ITEM (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + 2)
#define AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_4_ITEM (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + 3)

#define AUX_GPIO_OUTLET_DUTY_PROPERTY          (PRIVATE_DATA->gpio_outlet_duty_property)
#define AUX_GPIO_OUTLET_DUTY_OUTLET_1_ITEM     (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + 0)
#define AUX_GPIO_OUTLET_DUTY_OUTLET_2_ITEM     (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + 1)
#define AUX_GPIO_OUTLET_DUTY_OUTLET_3_ITEM     (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + 2)
#define AUX_GPIO_OUTLET_DUTY_OUTLET_4_ITEM     (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + 3)

	typedef struct {

} logical_device_data;

typedef struct {
	int handle;
	int count_open;
	bool udp;
	pthread_mutex_t port_mutex;
	//bool pwm_present;
	bool pwm_enabled[4];
	int pwm_freq[4];
	int pwm_duty[4];

	bool relay_active[4];
	indigo_timer *relay_timers[4];
	pthread_mutex_t relay_mutex;
	indigo_timer *pwm_settings_timer;

	indigo_property *outlet_names_property,
					*outlet_types_property,
	                *gpio_outlet_property,
	                *gpio_outlet_pulse_property,
	                *gpio_outlet_frequencies_property,
	                *gpio_outlet_duty_property;
} asiair_private_data;

typedef struct {
	indigo_device *device;
	asiair_private_data *private_data;
} asiair_device_data;

static asiair_device_data device_data = {0};

static void create_device();
static void delete_device();

static int pi = -1; /* pigpio handle */
static int output_pins[] = {12, 13, 26, 18};

static bool asiair_connect_pigpiod() {
	pi = pigpio_start(NULL, NULL);
	struct stat sb;
	if (pi >= 0) {
		INDIGO_DRIVER_DEBUG(DRIVER_NAME, "pigpiod is running.");
		return true;
	} else {
		INDIGO_DRIVER_DEBUG(DRIVER_NAME, "pigpiod not running.");
		return false;
	}
}

static bool asiair_set_pin_mode(int pin, bool output)
{
	if (output)
	{
		if (set_mode(pi, pin, PI_OUTPUT) < 0)
		{
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set gpio%d for output", pin);
			return false;
		}
		if (set_pull_up_down(pi, pin, PI_PUD_OFF) < 0)
		{
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set gpio%d pull up/down", pin);
			return false;
		}
	}
	else
	{
		if (set_mode(pi, pin, PI_INPUT) < 0)
		{
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set gpio%d for input", pin);
			return false;
		}
		if (set_pull_up_down(pi, pin, PI_PUD_UP) < 0)
		{
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set gpio%d pull up/down", pin);
			return false;
		}
	}
	return true;
}

bool asiair_init_pins()
{
	// set pins to output mode
	for (int i = 0; i < 4; i++)
	{
		if (!asiair_set_pin_mode(output_pins[i], true))
			return false;
	}
	indigo_usleep(1000);
	return true;
}

bool asiair_deinit_pins()
{
	// set pins to input mode
	for (int i = 0; i < 4; i++)
	{
		if (!asiair_set_pin_mode(output_pins[i], false))
			return false;
	}
	indigo_usleep(1000);
	return true;
}

static bool asiair_pwm_get(int pin, int *pwm_frequency, int *duty_cycle) {
	*pwm_frequency = get_PWM_frequency(pi, pin); // frequency in Hz
	if (*pwm_frequency < 0) {
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to get PWM pwm_frequency for pin %d", pin);
		return false;
	}
	*duty_cycle = get_PWM_dutycycle(pi, pin);
	if (*duty_cycle == PI_NOT_PWM_GPIO) {
		*duty_cycle = gpio_read(pi, pin);
		if (*duty_cycle < 0) {
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to get PWM duty cycle for pin %d, error %d", pin, *duty_cycle);
			return false;
		}
		*duty_cycle = *duty_cycle * 100;
	}
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "asiair_pwm_get freq = %d, duty = %d, gpio = %d", *pwm_frequency, *duty_cycle, pin);
	return true;
}

static bool asiair_pwm_read(int pin, int *value)
{
	if (value == NULL)
		return false;

	int frequency, duty_cycle;
	if (!asiair_pwm_get(pin, &frequency, &duty_cycle)) return false;
	*value = duty_cycle;
	return true;
}

static bool asiair_pwm_write(int pin, int pwm_frequency, int duty_cycle)
{
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "PWM set frequency = %d, duty_cycle = %d gpio = %d", pwm_frequency, duty_cycle, pin);
	if (set_PWM_range(pi, pin, 100) < 0)
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set PWM range for pin %d", pin);
		return false;
	}
	if (set_PWM_frequency(pi, pin, pwm_frequency) < 0)
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set PWM frequency for pin %d", pin);
		return false;
	}
	int status = set_PWM_dutycycle(pi, pin, duty_cycle);
	if (status < 0)
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to set PWM duty cycle for pin %d, error %d", pin, status);
		return false;
	}
	return true;
}

static bool asiair_pin_read(int pin, int *value)
{
	if (value == NULL)
		return false;

	*value = gpio_read(pi, pin);
	if (*value < 0)
		/* PI_BAD_GPIO */
		{
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to read value from pin %d!", pin);
			return false;
		}
	return true;
}

static bool asiair_pin_write(int pin, int value) {
	if (gpio_write(pi, pin, value) < 0)
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "Failed to write value to pin %d!", pin);
		return false;
	}
	return true;
}

static bool asiair_set_output_line(indigo_device *device, uint16_t line, int value)
{
	if (line >= 4) return false;
	if (PRIVATE_DATA->pwm_enabled[line])
	{
		int duty = 0;
		if (value>0)
			duty = (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + line)->number.value;
		return asiair_pwm_write(output_pins[line], (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + line)->number.value, duty);
		//return asiair_pwm_write(output_pins[line], PRIVATE_DATA->pwm_freq[line], value);
	}
	else
	{
		return asiair_pin_write(output_pins[line], value);
	}
}

static bool asiair_read_output_lines(indigo_device * device, int *relay)
{
	for (int i = 0; i < 4; i++) {
		if (PRIVATE_DATA->pwm_enabled[i]) {
			if (!asiair_pwm_read(output_pins[i], &relay[i])) return false;
			if (relay[i] > 0) relay[i] = 1; /*when PWM is enabled set relay to "on"*/
		} else {
			if (!asiair_pin_read(output_pins[i], &relay[i])) return false;
		}
	}
	return true;
}

// --------------------------------------------------------------------------------- INDIGO AUX RELAYS device implementation

static int asiair_init_properties(indigo_device *device) {
	// -------------------------------------------------------------------------------- SIMULATION
	SIMULATION_PROPERTY->hidden = true;
	// -------------------------------------------------------------------------------- DEVICE_PORT
	DEVICE_PORT_PROPERTY->hidden = true;
	// --------------------------------------------------------------------------------
	INFO_PROPERTY->count = 5;
	// -------------------------------------------------------------------------------- OUTLET_NAMES
	AUX_OUTLET_NAMES_PROPERTY = indigo_init_text_property(NULL, device->name, AUX_OUTLET_NAMES_PROPERTY_NAME, AUX_RELAYS_GROUP, "Customize Output names", INDIGO_OK_STATE, INDIGO_RW_PERM, 4);
	if (AUX_OUTLET_NAMES_PROPERTY == NULL)
		return INDIGO_FAILED;
	indigo_init_text_item(AUX_OUTLET_NAME_1_ITEM, AUX_GPIO_OUTLET_NAME_1_ITEM_NAME, "Output 1", "Power #1");
	indigo_init_text_item(AUX_OUTLET_NAME_2_ITEM, AUX_GPIO_OUTLET_NAME_2_ITEM_NAME, "Output 2", "Power #2");
	indigo_init_text_item(AUX_OUTLET_NAME_3_ITEM, AUX_GPIO_OUTLET_NAME_3_ITEM_NAME, "Output 3", "Power #3");
	indigo_init_text_item(AUX_OUTLET_NAME_4_ITEM, AUX_GPIO_OUTLET_NAME_4_ITEM_NAME, "Output 4", "Power #4");
	// -------------------------------------------------------------------------------- GPIO OUTLETS
	AUX_OUTLET_TYPES_PROPERTY = indigo_init_switch_property(NULL, device->name, AUX_OUTLET_TYPES_PROPERTY_NAME, AUX_RELAYS_GROUP, "Enable PWM", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_ANY_OF_MANY_RULE,4);
	if (AUX_OUTLET_TYPES_PROPERTY == NULL)
		return INDIGO_FAILED;
	indigo_init_text_item(AUX_OUTLET_TYPE_1_ITEM, AUX_OUTLET_TYPE_1_ITEM_NAME, "PWM 1", false);
	indigo_init_text_item(AUX_OUTLET_TYPE_2_ITEM, AUX_OUTLET_TYPE_2_ITEM_NAME, "PWM 2", false);
	indigo_init_text_item(AUX_OUTLET_TYPE_3_ITEM, AUX_OUTLET_TYPE_3_ITEM_NAME, "PWM 3", false);
	indigo_init_text_item(AUX_OUTLET_TYPE_4_ITEM, AUX_OUTLET_TYPE_4_ITEM_NAME, "PWM 4", false);
	// -------------------------------------------------------------------------------- GPIO OUTLETS
	AUX_GPIO_OUTLET_PROPERTY = indigo_init_switch_property(NULL, device->name, AUX_GPIO_OUTLETS_PROPERTY_NAME, AUX_RELAYS_GROUP, "Outputs", INDIGO_OK_STATE, INDIGO_RW_PERM, INDIGO_ANY_OF_MANY_RULE, 4);
	if (AUX_GPIO_OUTLET_PROPERTY == NULL)
		return INDIGO_FAILED;
	indigo_init_switch_item(AUX_GPIO_OUTLET_1_ITEM, AUX_GPIO_OUTLETS_OUTLET_1_ITEM_NAME, "Power #1", false);
	indigo_init_switch_item(AUX_GPIO_OUTLET_2_ITEM, AUX_GPIO_OUTLETS_OUTLET_2_ITEM_NAME, "Power #2", false);
	indigo_init_switch_item(AUX_GPIO_OUTLET_3_ITEM, AUX_GPIO_OUTLETS_OUTLET_3_ITEM_NAME, "Power #3", false);
	indigo_init_switch_item(AUX_GPIO_OUTLET_4_ITEM, AUX_GPIO_OUTLETS_OUTLET_4_ITEM_NAME, "Power #4", false);
	// -------------------------------------------------------------------------------- GPIO PULSE OUTLETS
	AUX_OUTLET_PULSE_LENGTHS_PROPERTY = indigo_init_number_property(NULL, device->name, AUX_OUTLET_PULSE_LENGTHS_PROPERTY_NAME, AUX_RELAYS_GROUP, "Output pulse lengths (ms)", INDIGO_OK_STATE, INDIGO_RW_PERM, 4);
	if (AUX_OUTLET_PULSE_LENGTHS_PROPERTY == NULL)
		return INDIGO_FAILED;
	indigo_init_number_item(AUX_OUTLET_PULSE_LENGTHS_1_ITEM, AUX_GPIO_OUTLETS_OUTLET_1_ITEM_NAME, "Output #1", 0, 100000, 100, 0);
	indigo_init_number_item(AUX_OUTLET_PULSE_LENGTHS_2_ITEM, AUX_GPIO_OUTLETS_OUTLET_2_ITEM_NAME, "Output #2", 0, 100000, 100, 0);
	indigo_init_number_item(AUX_OUTLET_PULSE_LENGTHS_3_ITEM, AUX_GPIO_OUTLETS_OUTLET_3_ITEM_NAME, "Output #3", 0, 100000, 100, 0);
	indigo_init_number_item(AUX_OUTLET_PULSE_LENGTHS_4_ITEM, AUX_GPIO_OUTLETS_OUTLET_4_ITEM_NAME, "Output #4", 0, 100000, 100, 0);
	// -------------------------------------------------------------------------------- AUX_GPIO_OUTLET_FREQUENCIES
	AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY = indigo_init_number_property(NULL, device->name, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY_NAME, AUX_RELAYS_GROUP, "PWM Frequencies (Hz)", INDIGO_OK_STATE, INDIGO_RW_PERM, 4);
	if (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY == NULL)
		return INDIGO_FAILED;
	indigo_init_number_item(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_1_ITEM, AUX_GPIO_OUTLETS_OUTLET_1_ITEM_NAME, "Output #1", 0.5, 1000000, 100, 100);
	indigo_init_number_item(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_2_ITEM, AUX_GPIO_OUTLETS_OUTLET_2_ITEM_NAME, "Output #2", 0.5, 1000000, 100, 100);
	indigo_init_number_item(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_3_ITEM, AUX_GPIO_OUTLETS_OUTLET_3_ITEM_NAME, "Output #3", 0.5, 1000000, 100, 100);
	indigo_init_number_item(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_4_ITEM, AUX_GPIO_OUTLETS_OUTLET_4_ITEM_NAME, "Output #4", 0.5, 1000000, 100, 100);
	// -------------------------------------------------------------------------------- AUX_GPIO_OUTLET_DUTY_CYCLES
	//AUX_GPIO_OUTLET_DUTY_PROPERTY = indigo_init_number_property(NULL, device->name, AUX_GPIO_OUTLET_DUTY_PROPERTY_NAME, AUX_RELAYS_GROUP, "PWM Duty cycles (%)", INDIGO_OK_STATE, INDIGO_RW_PERM, 4);
	AUX_GPIO_OUTLET_DUTY_PROPERTY = indigo_init_number_property(NULL, device->name, AUX_HEATER_OUTLET_PROPERTY_NAME, AUX_RELAYS_GROUP, "PWM Duty cycles (%)", INDIGO_OK_STATE, INDIGO_RW_PERM, 4);
	if (AUX_GPIO_OUTLET_DUTY_PROPERTY == NULL)
		return INDIGO_FAILED;
	indigo_init_number_item(AUX_GPIO_OUTLET_DUTY_OUTLET_1_ITEM, AUX_GPIO_OUTLETS_OUTLET_1_ITEM_NAME, "Output #1", 0, 100, 1, 100);
	indigo_init_number_item(AUX_GPIO_OUTLET_DUTY_OUTLET_2_ITEM, AUX_GPIO_OUTLETS_OUTLET_2_ITEM_NAME, "Output #2", 0, 100, 1, 100);
	indigo_init_number_item(AUX_GPIO_OUTLET_DUTY_OUTLET_3_ITEM, AUX_GPIO_OUTLETS_OUTLET_3_ITEM_NAME, "Output #3", 0, 100, 1, 100);
	indigo_init_number_item(AUX_GPIO_OUTLET_DUTY_OUTLET_4_ITEM, AUX_GPIO_OUTLETS_OUTLET_4_ITEM_NAME, "Output #4", 0, 100, 1, 100);
	//---------------------------------------------------------------------------
	return INDIGO_OK;
}

static bool update_pwm_properties(indigo_device *device)
{
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "update_pwm_properties called");

	int relay_value[4];
	if (!asiair_read_output_lines(device, relay_value))
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_read(%d) failed", PRIVATE_DATA->handle);
		return false;
	}

	int pwm_frequency, duty_cycle;
	for (int i = 0; i < 4; i++)
	{
		if ((relay_value[i] > 0) && PRIVATE_DATA->pwm_enabled[i])
		{
			if (!asiair_pwm_get(output_pins[i], &pwm_frequency, &duty_cycle))
			{
				AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->state = INDIGO_ALERT_STATE;
				AUX_GPIO_OUTLET_DUTY_PROPERTY->state = INDIGO_ALERT_STATE;
				return false;
			}
			else
			{
				(AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + i)->number.value = (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + i)->number.target = pwm_frequency;
				(AUX_GPIO_OUTLET_DUTY_PROPERTY->items + i)->number.value = (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + i)->number.target = duty_cycle;
			}
		}
	}

	indigo_update_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, NULL);
	indigo_update_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, NULL);
	return true;
}

// static void pwm_settings_timer_callback(indigo_device *device)
// {
// 	update_pwm_properties(device);
// 	indigo_reschedule_timer(device, 1, &PRIVATE_DATA->pwm_settings_timer);
// }

static void relay_1_timer_callback(indigo_device *device) {
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "relay timer callback called");
	pthread_mutex_lock(&PRIVATE_DATA->relay_mutex);
	PRIVATE_DATA->relay_active[0] = false;
	asiair_set_output_line(device, 0, 0);
	AUX_GPIO_OUTLET_1_ITEM->sw.value = false;
	indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
	pthread_mutex_unlock(&PRIVATE_DATA->relay_mutex);
}

static void relay_2_timer_callback(indigo_device *device) {
	pthread_mutex_lock(&PRIVATE_DATA->relay_mutex);
	PRIVATE_DATA->relay_active[1] = false;
	asiair_set_output_line(device, 1, 0);
	AUX_GPIO_OUTLET_2_ITEM->sw.value = false;
	indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
	pthread_mutex_unlock(&PRIVATE_DATA->relay_mutex);
}

static void relay_3_timer_callback(indigo_device *device) {
	pthread_mutex_lock(&PRIVATE_DATA->relay_mutex);
	PRIVATE_DATA->relay_active[2] = false;
	asiair_set_output_line(device, 2, 0);
	AUX_GPIO_OUTLET_3_ITEM->sw.value = false;
	indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
	pthread_mutex_unlock(&PRIVATE_DATA->relay_mutex);
}

static void relay_4_timer_callback(indigo_device *device) {
	pthread_mutex_lock(&PRIVATE_DATA->relay_mutex);
	PRIVATE_DATA->relay_active[3] = false;
	asiair_set_output_line(device, 3, 0);
	AUX_GPIO_OUTLET_4_ITEM->sw.value = false;
	indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
	pthread_mutex_unlock(&PRIVATE_DATA->relay_mutex);
}



static void (*relay_timer_callbacks[])(indigo_device*) = {
	relay_1_timer_callback,
	relay_2_timer_callback,
	relay_3_timer_callback,
	relay_4_timer_callback,
};

static bool set_pwm_properties(indigo_device *device)
{
	int relay_value[4];

	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_pwm_properties called");

	if (!asiair_read_output_lines(device, relay_value))
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_read(%d) failed", PRIVATE_DATA->handle);
		return false;
	}

	for (int i = 0; i < 4; i++)
	{
		if ((relay_value[i] > 0) && PRIVATE_DATA->pwm_enabled[i])
		{
			if (!asiair_pwm_write(output_pins[i], (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + i)->number.value, (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + i)->number.value)) return false;
		}
	}
	return update_pwm_properties(device);
}

static bool set_gpio_outlets(indigo_device *device)
{
	bool success = true;
	int relay_value[4];
	
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_gpio_outlets called");

	if (!asiair_read_output_lines(device, relay_value))
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_read(%d) failed", PRIVATE_DATA->handle);
		return false;
	}

	for (int i = 0; i < 4; i++)
	{
		if ((AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value != relay_value[i])
		{
			if (((AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + i)->number.value > 0) && (AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value && !PRIVATE_DATA->relay_active[i])
			{
				INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_gpio_outlets called, in relay timer workflow");
				if (!asiair_set_output_line(device, i, (int)(AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value))
				{
					INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_write(%d) failed, did you authorize?", PRIVATE_DATA->handle);
					success = false;
				}
				else
				{
					PRIVATE_DATA->relay_active[i] = true;
					indigo_set_timer(device, ((AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + i)->number.value) / 1000.0, relay_timer_callbacks[i], &PRIVATE_DATA->relay_timers[i]);
				}
			}
			else if ((AUX_OUTLET_PULSE_LENGTHS_PROPERTY->items + i)->number.value == 0 || (!(AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value && !PRIVATE_DATA->relay_active[i]))
			{
				INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_gpio_outlets called, standard workflow switch = %d", (int)(AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value);
				if (!asiair_set_output_line(device, i, (int)(AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value))
				{
					INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_write(%d) failed, did you authorize?", PRIVATE_DATA->handle);
					success = false;
				}
			}
		}
	}
	return success && update_pwm_properties(device);
}

static bool set_gpio_types(indigo_device *device)
{
	int relay_value[4];

	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_gpio_types called");

	if (!asiair_read_output_lines(device, relay_value))
	{
		INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_read(%d) failed", PRIVATE_DATA->handle);
		return false;
	}

	for (int i = 0; i < 4; i++)
	{
		if (PRIVATE_DATA->pwm_enabled[i] != (AUX_OUTLET_TYPES_PROPERTY->items + i)->sw.value) {
			PRIVATE_DATA->pwm_enabled[i] = (AUX_OUTLET_TYPES_PROPERTY->items + i)->sw.value;
			if (!PRIVATE_DATA->pwm_enabled[i])
			{
				//was PWM, now set to digital
				INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_gpio_types called, disable PWM on pin %d", i);
				if (relay_value[i] > 0)
				{
					asiair_pin_write(output_pins[i], 0); //disable PWM
					(AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value = 0;
				}
			}
			else
			{
				INDIGO_DRIVER_DEBUG(DRIVER_NAME, "set_gpio_types called, enable PWM on pin %d", i);
				//was digital, now set to PWM when relay is on
				if (relay_value[i] > 0)
				{
					if (!asiair_pwm_write(output_pins[i], (AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + i)->number.value, (AUX_GPIO_OUTLET_DUTY_PROPERTY->items + i)->number.value))
						return false;
				}
			}
		}
	}
	return update_pwm_properties(device);
}

static indigo_result aux_enumerate_properties(indigo_device *device, indigo_client *client, indigo_property *property) {
	if (IS_CONNECTED) {
		indigo_define_matching_property(AUX_OUTLET_TYPES_PROPERTY);
		indigo_define_matching_property(AUX_GPIO_OUTLET_PROPERTY);
		indigo_define_matching_property(AUX_OUTLET_PULSE_LENGTHS_PROPERTY);
		indigo_define_matching_property(AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY);
		indigo_define_matching_property(AUX_GPIO_OUTLET_DUTY_PROPERTY);
	}
	indigo_define_matching_property(AUX_OUTLET_NAMES_PROPERTY);

	return indigo_aux_enumerate_properties(device, NULL, NULL);
}


static indigo_result aux_attach(indigo_device *device) {
	assert(device != NULL);
	assert(PRIVATE_DATA != NULL);
	if (indigo_aux_attach(device, DRIVER_NAME, DRIVER_VERSION, INDIGO_INTERFACE_AUX_GPIO) == INDIGO_OK) {
		pthread_mutex_init(&PRIVATE_DATA->relay_mutex, NULL);
		// --------------------------------------------------------------------------------
		if (asiair_init_properties(device) != INDIGO_OK) return INDIGO_FAILED;
		INDIGO_DEVICE_ATTACH_LOG(DRIVER_NAME, device->name);
		return aux_enumerate_properties(device, NULL, NULL);
	}
	return INDIGO_FAILED;
}


static void handle_aux_connect_property(indigo_device *device) {
	if (CONNECTION_CONNECTED_ITEM->sw.value) {
		if (asiair_connect_pigpiod()) {
			INDIGO_DRIVER_DEBUG(DRIVER_NAME, "connected after pigpiod connection");
			AUX_GPIO_OUTLET_DUTY_PROPERTY->hidden = false;
			AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->hidden = false;
			if (asiair_init_pins()) {
				char board[INDIGO_VALUE_SIZE] = "N/A";
				char firmware[INDIGO_VALUE_SIZE] = "N/A";
				indigo_copy_value(INFO_DEVICE_MODEL_ITEM->text.value, board);
				indigo_copy_value(INFO_DEVICE_FW_REVISION_ITEM->text.value, firmware);
				indigo_update_property(device, INFO_PROPERTY, NULL);

				// Initialize to current state of outlets
				// TODO: PWM settings of active lines (Inactive lines loose PWM info.
				int relay_value[4];
				if (!asiair_read_output_lines(device, relay_value)) {
					INDIGO_DRIVER_ERROR(DRIVER_NAME, "asiair_pin_read(%d) failed", PRIVATE_DATA->handle);
					AUX_GPIO_OUTLET_PROPERTY->state = INDIGO_ALERT_STATE;
				} else {
					int pwm_frequency, duty_cycle;
					for (int i = 0; i < 4; i++) {
						(AUX_OUTLET_TYPES_PROPERTY->items + i)->sw.value = 0;
						(AUX_GPIO_OUTLET_PROPERTY->items + i)->sw.value = relay_value[i];
						PRIVATE_DATA->relay_active[i] = false;

						//PRIVATE_DATA->pwm_freq[i] = (int)(AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->items + i)->number.target;
						//PRIVATE_DATA->pwm_duty[i] = (int)(AUX_GPIO_OUTLET_DUTY_PROPERTY->items + i)->number.target;
						//INDIGO_DRIVER_DEBUG(DRIVER_NAME, "connected before write, TODO check if needed");
						//asiair_pwm_write(output_pins[i], PRIVATE_DATA->pwm_freq[i], PRIVATE_DATA->pwm_duty[i]);
					}
				}

				indigo_define_property(device, AUX_OUTLET_TYPES_PROPERTY, NULL);
				indigo_define_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
				indigo_define_property(device, AUX_OUTLET_PULSE_LENGTHS_PROPERTY, NULL);
				indigo_define_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, NULL);
				indigo_define_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, NULL);

				//indigo_set_timer(device, 0, pwm_settings_timer_callback, &PRIVATE_DATA->pwm_settings_timer);

				CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
			} else {
				CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
				indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, false);
			}
		}
		else
		{
			CONNECTION_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, false);
			INDIGO_DRIVER_ERROR(DRIVER_NAME, "Process pigpiod not running! Use 'sudo pigpiod' to start it.");
		}	
	} else {
		indigo_update_property(device, CONNECTION_PROPERTY, NULL);
		for (int i = 0; i < 4; i++) {
			indigo_cancel_timer_sync(device, &PRIVATE_DATA->relay_timers[i]);
		}
		//indigo_cancel_timer_sync(device, &PRIVATE_DATA->pwm_settings_timer);
		asiair_deinit_pins();
		indigo_delete_property(device, AUX_OUTLET_TYPES_PROPERTY, NULL);
		indigo_delete_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
		indigo_delete_property(device, AUX_OUTLET_PULSE_LENGTHS_PROPERTY, NULL);
		indigo_delete_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, NULL);
		indigo_delete_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, NULL);
		CONNECTION_PROPERTY->state = INDIGO_OK_STATE;
	}

	indigo_aux_change_property(device, NULL, CONNECTION_PROPERTY);
}

static indigo_result aux_change_property(indigo_device *device, indigo_client *client, indigo_property *property) {
	assert(device != NULL);
	assert(DEVICE_CONTEXT != NULL);
	assert(property != NULL);
	if (indigo_property_match_changeable(CONNECTION_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- CONNECTION
		if (indigo_ignore_connection_change(device, property))
			return INDIGO_OK;
		indigo_property_copy_values(CONNECTION_PROPERTY, property, false);
		CONNECTION_PROPERTY->state = INDIGO_BUSY_STATE;
		indigo_update_property(device, CONNECTION_PROPERTY, NULL);
		indigo_set_timer(device, 0, handle_aux_connect_property, NULL);
		return INDIGO_OK;
	} else if (indigo_property_match_changeable(AUX_OUTLET_NAMES_PROPERTY, property)) {
		// -------------------------------------------------------------------------------- AUX_OUTLET_NAMES
		indigo_property_copy_values(AUX_OUTLET_NAMES_PROPERTY, property, false);
		if (IS_CONNECTED) {
			indigo_delete_property(device, AUX_OUTLET_TYPES_PROPERTY, NULL);
			indigo_delete_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
			indigo_delete_property(device, AUX_OUTLET_PULSE_LENGTHS_PROPERTY, NULL);
			indigo_delete_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, NULL);
			indigo_delete_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, NULL);
		}
		snprintf(AUX_OUTLET_TYPE_1_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_1_ITEM->text.value);
		snprintf(AUX_OUTLET_TYPE_2_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_2_ITEM->text.value);
		snprintf(AUX_OUTLET_TYPE_3_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_3_ITEM->text.value);
		snprintf(AUX_OUTLET_TYPE_4_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_4_ITEM->text.value);

		snprintf(AUX_GPIO_OUTLET_1_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_1_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_2_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_2_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_3_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_3_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_4_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_4_ITEM->text.value);

		snprintf(AUX_OUTLET_PULSE_LENGTHS_1_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_1_ITEM->text.value);
		snprintf(AUX_OUTLET_PULSE_LENGTHS_2_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_2_ITEM->text.value);
		snprintf(AUX_OUTLET_PULSE_LENGTHS_3_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_3_ITEM->text.value);
		snprintf(AUX_OUTLET_PULSE_LENGTHS_4_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_4_ITEM->text.value);

		snprintf(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_1_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_1_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_2_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_2_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_3_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_3_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_FREQUENCIES_OUTLET_4_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_4_ITEM->text.value);

		snprintf(AUX_GPIO_OUTLET_DUTY_OUTLET_1_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_1_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_DUTY_OUTLET_2_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_2_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_DUTY_OUTLET_3_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_3_ITEM->text.value);
		snprintf(AUX_GPIO_OUTLET_DUTY_OUTLET_4_ITEM->label, INDIGO_NAME_SIZE, "%s", AUX_OUTLET_NAME_4_ITEM->text.value);

		AUX_OUTLET_NAMES_PROPERTY->state = INDIGO_OK_STATE;
		if (IS_CONNECTED) {
			indigo_define_property(device, AUX_OUTLET_TYPES_PROPERTY, NULL);
			indigo_define_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
			indigo_define_property(device, AUX_OUTLET_PULSE_LENGTHS_PROPERTY, NULL);
			indigo_define_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, NULL);
			indigo_define_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, NULL);
		}
		indigo_update_property(device, AUX_OUTLET_NAMES_PROPERTY, NULL);
		return INDIGO_OK;
	}
	else if (indigo_property_match_changeable(AUX_OUTLET_TYPES_PROPERTY, property))
	{
		// -------------------------------------------------------------------------------- AUX_OUTLET_TYPES
		indigo_property_copy_values(AUX_OUTLET_TYPES_PROPERTY, property, false);
		if (set_gpio_types(device) == true)
		{
			AUX_OUTLET_TYPES_PROPERTY->state = INDIGO_OK_STATE;
			indigo_update_property(device, AUX_OUTLET_TYPES_PROPERTY, NULL);
			indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL); /* disabling PWM turns relay off*/
		}
		else
		{
			AUX_OUTLET_TYPES_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, AUX_OUTLET_TYPES_PROPERTY, "Output operation failed");
		}
		return INDIGO_OK;
	}
	else if (indigo_property_match_changeable(AUX_GPIO_OUTLET_PROPERTY, property))
	{
		// -------------------------------------------------------------------------------- AUX_GPIO_OUTLET
		indigo_property_copy_values(AUX_GPIO_OUTLET_PROPERTY, property, false);
		if (set_gpio_outlets(device) == true)
		{
			AUX_GPIO_OUTLET_PROPERTY->state = INDIGO_OK_STATE;
			indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, NULL);
		}
		else
		{
			AUX_GPIO_OUTLET_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, AUX_GPIO_OUTLET_PROPERTY, "Output operation failed.");
		}
		return INDIGO_OK;
	}
	else if (indigo_property_match_changeable(AUX_OUTLET_PULSE_LENGTHS_PROPERTY, property))
	{
		// -------------------------------------------------------------------------------- AUX_OUTLET_PULSE_LENGTHS
		indigo_property_copy_values(AUX_OUTLET_PULSE_LENGTHS_PROPERTY, property, false);
		indigo_update_property(device, AUX_OUTLET_PULSE_LENGTHS_PROPERTY, NULL);
		return INDIGO_OK;
	}
	else if (indigo_property_match_changeable(AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, property))
	{
		// -------------------------------------------------------------------------------- AUX_GPIO_OUTLET_FREQUENCIES
		indigo_property_copy_values(AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, property, false);
		if (set_pwm_properties(device) == true)
		{
			AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->state = INDIGO_OK_STATE;
			indigo_update_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, NULL);
		}
		else
		{
			AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY, "Output operation failed");
		}
		return INDIGO_OK;
	}
	else if (indigo_property_match_changeable(AUX_GPIO_OUTLET_DUTY_PROPERTY, property))
	{
		// -------------------------------------------------------------------------------- AUX_GPIO_OUTLET_DUTY_PROPERTY
		indigo_property_copy_values(AUX_GPIO_OUTLET_DUTY_PROPERTY, property, false);
		if (set_pwm_properties(device) == true)
		{
			AUX_GPIO_OUTLET_DUTY_PROPERTY->state = INDIGO_OK_STATE;
			indigo_update_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, NULL);
		}
		else
		{
			AUX_GPIO_OUTLET_DUTY_PROPERTY->state = INDIGO_ALERT_STATE;
			indigo_update_property(device, AUX_GPIO_OUTLET_DUTY_PROPERTY, "Output operation failed");
		}
		return INDIGO_OK;
	}
	else if (indigo_property_match_changeable(CONFIG_PROPERTY, property))
	{
		// -------------------------------------------------------------------------------- CONFIG
		if (indigo_switch_match(CONFIG_SAVE_ITEM, property)) {
			indigo_save_property(device, NULL, AUX_OUTLET_NAMES_PROPERTY);
			indigo_save_property(device, NULL, AUX_OUTLET_TYPES_PROPERTY);
			indigo_save_property(device, NULL, AUX_GPIO_OUTLET_PROPERTY);
			indigo_save_property(device, NULL, AUX_OUTLET_PULSE_LENGTHS_PROPERTY);
			indigo_save_property(device, NULL, AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY);
			indigo_save_property(device, NULL, AUX_GPIO_OUTLET_DUTY_PROPERTY);
		}
	}
	// --------------------------------------------------------------------------------
	return indigo_aux_change_property(device, client, property);
}


static indigo_result aux_detach(indigo_device *device) {
	assert(device != NULL);
	if (IS_CONNECTED) {
		indigo_set_switch(CONNECTION_PROPERTY, CONNECTION_DISCONNECTED_ITEM, true);
		handle_aux_connect_property(device);
	}
	indigo_release_property(AUX_OUTLET_TYPES_PROPERTY);
	indigo_release_property(AUX_GPIO_OUTLET_PROPERTY);
	indigo_release_property(AUX_OUTLET_PULSE_LENGTHS_PROPERTY);
	indigo_release_property(AUX_GPIO_OUTLET_FREQUENCIES_PROPERTY);
	indigo_release_property(AUX_GPIO_OUTLET_DUTY_PROPERTY);
	INDIGO_DEVICE_DETACH_LOG(DRIVER_NAME, device->name);

	indigo_delete_property(device, AUX_OUTLET_NAMES_PROPERTY, NULL);
	indigo_release_property(AUX_OUTLET_NAMES_PROPERTY);

	return indigo_aux_detach(device);
}

// --------------------------------------------------------------------------------

//static int device_number = 0;

static void create_device() {
	static indigo_device aux_template = INDIGO_DEVICE_INITIALIZER(
		AUX_ASIAIR_NAME,
		aux_attach,
		aux_enumerate_properties,
		aux_change_property,
		NULL,
		aux_detach
	);

	if (device_data.device != NULL) return;

	if (device_data.private_data == NULL) {
		device_data.private_data = indigo_safe_malloc(sizeof(asiair_private_data));
		pthread_mutex_init(&device_data.private_data->port_mutex, NULL);
		INDIGO_DRIVER_DEBUG(DRIVER_NAME, "ADD: PRIVATE_DATA");
	}

	device_data.device = indigo_safe_malloc_copy(sizeof(indigo_device), &aux_template);
	sprintf(device_data.device->name, "%s", AUX_ASIAIR_NAME);

	device_data.device->private_data = device_data.private_data;
	indigo_attach_device(device_data.device);
	INDIGO_DRIVER_DEBUG(DRIVER_NAME, "ADD: Device.");
}

static void delete_device() {
	if (device_data.device != NULL) {
		indigo_detach_device(device_data.device);
		INDIGO_DRIVER_DEBUG(DRIVER_NAME, "REMOVE: Device.");
		free(device_data.device);
		device_data.device = NULL;
	}

	if (device_data.private_data != NULL) {
		free(device_data.private_data);
		device_data.private_data = NULL;
		INDIGO_DRIVER_DEBUG(DRIVER_NAME, "REMOVE: PRIVATE_DATA");
	}
}


indigo_result indigo_aux_asiair(indigo_driver_action action, indigo_driver_info *info) {

	static indigo_driver_action last_action = INDIGO_DRIVER_SHUTDOWN;

	SET_DRIVER_INFO(info, DRIVER_INFO, __FUNCTION__, DRIVER_VERSION, false, last_action);

	if (action == last_action)
		return INDIGO_OK;

	switch (action) {
	case INDIGO_DRIVER_INIT:
		last_action = action;
		create_device();
		break;

	case INDIGO_DRIVER_SHUTDOWN:
		VERIFY_NOT_CONNECTED(device_data.device);
		last_action = action;
		delete_device();
		break;

	case INDIGO_DRIVER_INFO:
		break;
	}

	return INDIGO_OK;
}

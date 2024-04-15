/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
//#define TEST_MAXIM_ALGORITHM 1

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_imu_sens.h"

#include "driver/i2c.h"

#include "algorithm_by_RF.h"
#include "MAX30102.h"

#ifdef TEST_MAXIM_ALGORITHM
#include "algorithm.h"
#endif

#define BLINK_GPIO 2 // pico-d4

#define MAX30102_INT 25

static uint8_t s_led_state = 0;

static const char *tag = "NimBLE_BLE_IMU";

static TimerHandle_t ble_imu_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

static const char *device_name = "ESP32";

static int ble_imu_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t ble_imu_addr_type;

uint16_t update_nofity_period = 0;	// ms

TaskHandle_t sensor_reader_handle = NULL;

int32_t n_heart_rate; //heart rate value

#define DELAY_AMOSTRAGEM 40

#ifdef __cplusplus
extern "C" {
#endif
	void mpu6050_task(void *pvParameters);
	void mpu6050_get_gravity(uint8_t *val);
	void mpu6050_get_gyro(uint8_t *val);
	void mpu6050_get_YawPitchRoll(uint8_t *val);
	void mpu6050_get_temperature(uint8_t *val);
#ifdef __cplusplus
}
#endif

/**
 * Utility function to log an array of bytes.
 */
void print_bytes(const uint8_t *bytes, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
	}
}

void print_addr(const void *addr)
{
	const uint8_t *u8p;

	u8p = addr;
	MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
				u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void blink_led(void)
{
	s_led_state = s_led_state ? 0 : 1;
	/* Set the GPIO level according to the state (LOW or HIGH)*/
	gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
	gpio_reset_pin(BLINK_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void	ble_imu_advertise(void)
{
	struct ble_gap_adv_params adv_params;
	struct ble_hs_adv_fields fields;
	int rc;

	/*
	 *  Set the advertisement data included in our advertisements:
	 *     o Flags (indicates advertisement type and other general info)
	 *     o Advertising tx power
	 *     o Device name
	 */
	memset(&fields, 0, sizeof(fields));

	/*
	 * Advertise two flags:
	 *      o Discoverability in forthcoming advertisement (general)
	 *      o BLE-only (BR/EDR unsupported)
	 */
	fields.flags = BLE_HS_ADV_F_DISC_GEN |
				   BLE_HS_ADV_F_BREDR_UNSUP;

	/*
	 * Indicate that the TX power level field should be included; have the
	 * stack fill this value automatically.  This is done by assigning the
	 * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
	 */
	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	fields.name = (uint8_t *)device_name;
	fields.name_len = strlen(device_name);
	fields.name_is_complete = 1;

// jht peter fix can not find heart rate device when using nrf toolbox 
//  fields.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(GATT_HRS_UUID) };
//  fields.num_uuids16 = 1;
//  fields.uuids16_is_complete = 1;

	rc = ble_gap_adv_set_fields(&fields);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
		return;
	}

	/* Begin advertising */
	memset(&adv_params, 0, sizeof(adv_params));
	adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
	rc = ble_gap_adv_start(ble_imu_addr_type, NULL, BLE_HS_FOREVER,
						   &adv_params, ble_imu_gap_event, NULL);
	if (rc != 0) {
		MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
		return;
	}
}

static void	ble_imu_tx_imu_stop(void)
{
	xTimerStop( ble_imu_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset imu measurement */
static void	ble_imu_tx_imu_reset(void)
{
	int rc;

	if (update_nofity_period >= 100) {
		ESP_LOGI(tag, "set notify period to %d ms", update_nofity_period);
		xTimerChangePeriod(ble_imu_tx_timer,  pdMS_TO_TICKS(update_nofity_period), 1000 / portTICK_PERIOD_MS);
		update_nofity_period = 0;
	}

	if (xTimerReset(ble_imu_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
		rc = 0;
	} else {
		rc = 1;
	}

	assert(rc == 0);

}

/* This function get imu and notifies it to the client */
static void	ble_imu_tx_imu(TimerHandle_t ev)
{
	static uint8_t val[20];
	int rc;
	struct os_mbuf *om;

	if (!notify_state) {
		ble_imu_tx_imu_stop();
		return;
	}

	blink_led();

	mpu6050_get_gravity(&val[0]);
	mpu6050_get_gyro(&val[6]);
	mpu6050_get_YawPitchRoll(&val[12]);

	val[18] = n_heart_rate & 0xFF;
	val[19] = n_heart_rate >> 8;

	om = ble_hs_mbuf_from_flat(val, sizeof(val));
	rc = ble_gatts_notify_custom(conn_handle, imu_handle, om);

	assert(rc == 0);

	ble_imu_tx_imu_reset();
}

static int ble_imu_gap_event(struct ble_gap_event *event, void *arg)
{
	switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
		/* A new connection was established or a connection attempt failed */
		MODLOG_DFLT(INFO, "connection %s; status=%d\n",
					event->connect.status == 0 ? "established" : "failed",
					event->connect.status);

		if (event->connect.status != 0) {
			/* Connection failed; resume advertising */
			ble_imu_advertise();
		}
		conn_handle = event->connect.conn_handle;
		break;

	case BLE_GAP_EVENT_DISCONNECT:
		MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

		/* Connection terminated; resume advertising */
		ble_imu_advertise();
		break;

	case BLE_GAP_EVENT_ADV_COMPLETE:
		MODLOG_DFLT(INFO, "adv complete\n");
		ble_imu_advertise();
		break;

	case BLE_GAP_EVENT_SUBSCRIBE:
		MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
					"val_handle=%d\n",
					event->subscribe.cur_notify, imu_handle);
		if (event->subscribe.attr_handle == imu_handle) {
			notify_state = event->subscribe.cur_notify;
			ble_imu_tx_imu_reset();
		} else if (event->subscribe.attr_handle != imu_handle) {
			notify_state = event->subscribe.cur_notify;
			ble_imu_tx_imu_stop();
		}
		ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
		break;

	case BLE_GAP_EVENT_MTU:
		MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
					event->mtu.conn_handle,
					event->mtu.value);
		break;

	}

	return 0;
}

static void	ble_imu_on_sync(void)
{
	int rc;

	rc = ble_hs_id_infer_auto(0, &ble_imu_addr_type);
	assert(rc == 0);

	uint8_t addr_val[6] = {0};
	rc = ble_hs_id_copy_addr(ble_imu_addr_type, addr_val, NULL);

	MODLOG_DFLT(INFO, "Device Address: ");
	print_addr(addr_val);
	MODLOG_DFLT(INFO, "\n");

	/* Begin advertising */
	ble_imu_advertise();
}

static void	ble_imu_on_reset(int reason)
{
	MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void ble_imu_host_task(void *param)
{
	ESP_LOGI(tag, "BLE Host Task Started");
	/* This function will return only when nimble_port_stop() is executed */
	nimble_port_run();

	nimble_port_freertos_deinit();
}

void start_i2c(void) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
	conf.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config((i2c_port_t)CONFIG_I2C_NUM, &conf));
	ESP_ERROR_CHECK(i2c_driver_install((i2c_port_t)CONFIG_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

void sensor_data_reader(void *pvParameters)
{
	uint32_t aun_ir_buffer[100]; //infrared LED sensor data
	uint32_t aun_red_buffer[100]; //red LED sensor data
	float n_spo2, ratio, correl; //SPO2 value

	int8_t ch_spo2_valid; //indicator to show if the SPO2 calculation is valid

	int8_t ch_hr_valid;	//indicator to show if the heart rate calculation is valid
	int32_t i;

	vTaskDelay(pdMS_TO_TICKS(100));
	max30102_Init();

	//read the first 100 samples, and determine the signal range
	for (i = 0; i < BUFFER_SIZE; i++) {
		while (gpio_get_level(MAX30102_INT)); //wait until the interrupt pin asserts

		// my max30102 module has ir, red inverted. modify max30102_ReadFifo((aun_red_buffer+i), (aun_ir_buffer+i));
		max30102_ReadFifo((aun_ir_buffer + i), (aun_red_buffer + i)); //read from MAX30102 FIFO
	}
	//calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
	rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);


	for (;;) {
		//dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
		for (i = 25; i < 100; i++) {
			aun_red_buffer[i - 25] = aun_red_buffer[i];
			aun_ir_buffer[i - 25] = aun_ir_buffer[i];
		}

		//take 25 sets of samples before calculating the heart rate.
		for (i = 75; i < 100; i++) {

			while (gpio_get_level(MAX30102_INT));
			max30102_ReadFifo((aun_ir_buffer + i), (aun_red_buffer + i)); //read from MAX30102 FIFO
		}
		
		rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);

#ifdef TEST_MAXIM_ALGORITHM
		//calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using MAXIM's method
		float n_spo2_maxim;	 //SPO2 value
		int8_t ch_spo2_valid_maxim;	//indicator to show if the SPO2 calculation is valid
		int32_t n_heart_rate_maxim;	//heart rate value
		int8_t  ch_hr_valid_maxim;	//indicator to show if the heart rate calculation is valid
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2_maxim, &ch_spo2_valid_maxim, &n_heart_rate_maxim, &ch_hr_valid_maxim); 

		if (ch_hr_valid_maxim && ch_spo2_valid_maxim) 
			printf("%ld Hr(maxim), %.1f Sp(maxim) \n\r",n_heart_rate_maxim,n_spo2_maxim);
#endif // TEST_MAXIM_ALGORITHM

		if (ch_hr_valid && ch_spo2_valid)
			printf("%ld Hr, %.1f Sp \n\r",n_heart_rate,n_spo2);

	}
}

void app_main(void)
{
	int rc;

	configure_led();

	gpio_reset_pin(MAX30102_INT);
	gpio_set_direction(MAX30102_INT, GPIO_MODE_INPUT);
	gpio_set_pull_mode(MAX30102_INT, GPIO_PULLUP_ONLY);

	// Initialize i2c
	start_i2c();

	/* Initialize NVS ??it is used to store PHY calibration data */
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ret = nimble_port_init();
	if (ret != ESP_OK) {
		MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
		return;
	}

	/* Initialize the NimBLE host configuration */
	ble_hs_cfg.sync_cb = ble_imu_on_sync;
	ble_hs_cfg.reset_cb = ble_imu_on_reset;

	/* name, period/time,  auto reload, timer ID, callback */
	ble_imu_tx_timer = xTimerCreate("ble_imu_tx_timer", pdMS_TO_TICKS(200), pdTRUE, (void *)0, ble_imu_tx_imu);

	rc = gatt_svr_init();
	assert(rc == 0);

	/* Set the default device name */
	rc = ble_svc_gap_device_name_set(device_name);
	assert(rc == 0);

	/* Start the task */
	nimble_port_freertos_init(ble_imu_host_task);

	// Start imu task
	xTaskCreate(&mpu6050_task, "IMU", 1024*8, NULL, 5, NULL);

	// start heart rate sensor
	xTaskCreatePinnedToCore(sensor_data_reader, "Data", 10240, NULL, 2, &sensor_reader_handle, 1);
}

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

#include "max30102_api.h"
#include "algorithm.h"

#define BLINK_GPIO 2 // pico-d4

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

int heart_rate;
int32_t red_data = 0;
int32_t ir_data = 0;
int32_t red_data_buffer[BUFFER_SIZE];
int32_t ir_data_buffer[BUFFER_SIZE];
double auto_correlationated_data[BUFFER_SIZE];

char *data = NULL;

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

#ifdef __cplusplus
extern "C" {
#endif
void start_i2c(void);
#ifdef __cplusplus
}
#endif


//Configuracao dos registradores do modo de funcionamento do sensor MAX30102. Basta colocar o valor binario da configuracao que o compilador fara o trabalho de setar os bits corretos.
max_config max30102_configuration = {

		.INT_EN_1.A_FULL_EN         = 1,
		.INT_EN_1.PPG_RDY_EN        = 1,
		.INT_EN_1.ALC_OVF_EN        = 0,
		.INT_EN_1.PROX_INT_EN       = 0,

		.INT_EN_2.DIE_TEMP_RDY_EN   = 0,

		.FIFO_WRITE_PTR.FIFO_WR_PTR = 0,

		.OVEF_COUNTER.OVF_COUNTER   = 0,

		.FIFO_READ_PTR.FIFO_RD_PTR  = 0,

		.FIFO_CONF.SMP_AVE          = 0b010,  //media de 4 valores
		.FIFO_CONF.FIFO_ROLLOVER_EN = 1,      //fifo rollover enable
		.FIFO_CONF.FIFO_A_FULL      = 0,      //0

		.MODE_CONF.SHDN             = 0,
		.MODE_CONF.RESET            = 0,
		.MODE_CONF.MODE             = 0b011,  //SPO2 mode

		.SPO2_CONF.SPO2_ADC_RGE     = 0b01,   //16384 nA(Escala do DAC)
		.SPO2_CONF.SPO2_SR          = 0b001,  //200 samples per second
		.SPO2_CONF.LED_PW           = 0b10,   //pulso de 215 uS do led.

		.LED1_PULSE_AMP.LED1_PA     = 0x24,   //CORRENTE DO LED1 25.4mA
		.LED2_PULSE_AMP.LED2_PA     = 0x24,   //CORRENTE DO LED2 25.4mA

		.PROX_LED_PULS_AMP.PILOT_PA = 0X7F,

		.MULTI_LED_CONTROL1.SLOT2   = 0,      //Desabilitado
		.MULTI_LED_CONTROL1.SLOT1   = 0,      //Desabilitado

		.MULTI_LED_CONTROL2.SLOT4   = 0,      //Desabilitado
		.MULTI_LED_CONTROL2.SLOT3   = 0,      //Desabilitado
};

/**
 * Utility function to log an array of bytes.
 */
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_addr(const void *addr)
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
static void
ble_imu_advertise(void)
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

static void
ble_imu_tx_imu_stop(void)
{
    xTimerStop( ble_imu_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset imu measurement */
static void
ble_imu_tx_imu_reset(void)
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
static void
ble_imu_tx_imu(TimerHandle_t ev)
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

	val[18] = heart_rate & 0xFF;
	val[19] = heart_rate >> 8;

    om = ble_hs_mbuf_from_flat(val, sizeof(val));
    rc = ble_gatts_notify_custom(conn_handle, imu_handle, om);

    assert(rc == 0);

    ble_imu_tx_imu_reset();
}

static int
ble_imu_gap_event(struct ble_gap_event *event, void *arg)
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

static void
ble_imu_on_sync(void)
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

static void
ble_imu_on_reset(int reason)
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
	// i2c_set_timeout
}

void fill_buffers_data()
{
	for(int i = 0; i < BUFFER_SIZE; i++){
		read_max30102_fifo(&red_data, &ir_data);
		ir_data_buffer[i] = ir_data;
		red_data_buffer[i] = red_data;
		//printf("%d\n", red_data);
		ir_data = 0;
		red_data = 0;
		vTaskDelay(pdMS_TO_TICKS(DELAY_AMOSTRAGEM));
	}
}


void sensor_data_reader(void *pvParameters)
{
    //uint8_t sta_mac[6] = {0};
	vTaskDelay(pdMS_TO_TICKS(100));
	max30102_init(&max30102_configuration);
	init_time_array();
	uint64_t ir_mean;
	uint64_t red_mean;
	float temperature;
	double r0_autocorrelation;
	//size_t size;

	for(;;){
		//vTaskDelay(pdMS_TO_TICKS(100));
		fill_buffers_data();
		temperature = get_max30102_temp();
		remove_dc_part(ir_data_buffer, red_data_buffer, &ir_mean, &red_mean);
		remove_trend_line(ir_data_buffer);
		remove_trend_line(red_data_buffer);
		double pearson_correlation = correlation_datay_datax(red_data_buffer, ir_data_buffer);
		heart_rate = calculate_heart_rate(ir_data_buffer, &r0_autocorrelation, auto_correlationated_data);

		printf("\n");
		printf("HEART_RATE %d\n", heart_rate);
		printf("correlacao %f\n", pearson_correlation);
		printf("Temperature %f\n", temperature);

		if(pearson_correlation >= 0.7){ //Os dois sinais IR e RED devem ser fortemente relacionados.
			double spo2 = spo2_measurement(ir_data_buffer, red_data_buffer, ir_mean, red_mean);
			printf("SPO2 %f\n", spo2);
	        //size = asprintf(&data, "{\"mac\": \"%02x%02x%02x%02x%02x%02x\", \"spo2\":%f, \"heart_rate\":%d}",MAC2STR(sta_mac), spo2, heart_rate);
			//mqtt_publish(data, size);
		}
		printf("\n");

#if 0
		for(int i = 0; i < BUFFER_SIZE; i++){
			printf("%ld", ir_data_buffer[i]);
			printf(" ");
			printf("%ld", red_data_buffer[i]);
			printf(" ");
			printf("%f\n", auto_correlationated_data[i]);
		}
#endif
	}
}

void app_main(void)
{
    int rc;

	configure_led();

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

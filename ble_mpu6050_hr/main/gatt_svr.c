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

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_imu_sens.h"

#define TAG     "BLE_IMU_GATT_SVC"

#define UUID_ACCELEROMETER_SERVICE 							BLE_UUID128_DECLARE(0xa8, 0xa9, 0xdf, 0x22, 0x19, 0xfa, 0x62, 0xa0, 0x0a, 0x47, 0x1d, 0x25, 0x53, 0x07, 0x5d, 0xe9)
#define UUID_ACCELEROMETER_SERVICE_CHARACTERISTIC_DATA 		BLE_UUID128_DECLARE(0xa8, 0xa9, 0xdf, 0x22, 0x19, 0xfa, 0x62, 0xa0, 0x0a, 0x47, 0x1d, 0x25, 0x4b, 0xca, 0x5d, 0xe9)
#define UUID_ACCELEROMETER_SERVICE_CHARACTERISTIC_PERIOD 	BLE_UUID128_DECLARE(0xa8, 0xa9, 0xdf, 0x22, 0x19, 0xfa, 0x62, 0xa0, 0x0a, 0x47, 0x1d, 0x25, 0x24, 0xfb, 0x5d, 0xe9)

static const char *manuf_name = "JHT";
static const char *model_num = "BLE IMU sensor";
uint16_t imu_handle, set_period_handle;

static int
gatt_svr_chr_access_imu(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: imu */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = UUID_ACCELEROMETER_SERVICE,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: imu */
                .uuid = UUID_ACCELEROMETER_SERVICE_CHARACTERISTIC_DATA,
                .access_cb = gatt_svr_chr_access_imu,
                .val_handle = &imu_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
	         	/* Characteristic: write notify period */
	         	.uuid = UUID_ACCELEROMETER_SERVICE_CHARACTERISTIC_PERIOD,
	         	.access_cb = gatt_svr_chr_access_imu,
	         	.val_handle = &set_period_handle,
	         	.flags = BLE_GATT_CHR_F_WRITE,
	        }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        /* Service: Device Information */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: * Manufacturer name */
                .uuid = BLE_UUID16_DECLARE(GATT_MANUFACTURER_NAME_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /* Characteristic: Model number string */
                .uuid = BLE_UUID16_DECLARE(GATT_MODEL_NUMBER_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        0, /* No more services */
    },
};

static int
gatt_svr_chr_access_imu(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
//  /* Sensor location, set to "Chest" */
//  static uint8_t body_sens_loc = 0x01;
//  uint16_t uuid;
//  int rc;
//
//  uuid = ble_uuid_u16(ctxt->chr->uuid);
//
//  if (uuid == GATT_HRS_BODY_SENSOR_LOC_UUID) {
//      rc = os_mbuf_append(ctxt->om, &body_sens_loc, sizeof(body_sens_loc));
//
//      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
//  }
//
//  assert(0);
//  return BLE_ATT_ERR_UNLIKELY;
//      MODLOG_DFLT(DEBUG, "gatt_svr_chr_access_imu \n" );
//  return 0;
    uint8_t data[10];
    uint8_t len;
    struct os_mbuf *om;

	switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "Characteristic write; conn_handle=%d", conn_handle);
        if ((attr_handle == set_period_handle) && (0 == ble_uuid_cmp(ctxt->chr->uuid, UUID_ACCELEROMETER_SERVICE_CHARACTERISTIC_PERIOD))) {
            om = ctxt->om;
            len = os_mbuf_len(om);
            len = len < sizeof(data) ? len : sizeof(data);
            assert(os_mbuf_copydata(om, 0, len, data) == 0);
            ESP_LOG_BUFFER_HEX(TAG, data, len);
			update_nofity_period = (data[1] << 8) |data[0];
            return 0;
        }
        goto unknown;

    default:
        goto unknown;
    }

unknown:
    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_MODEL_NUMBER_UUID) {
        rc = os_mbuf_append(ctxt->om, model_num, strlen(model_num));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (uuid == GATT_MANUFACTURER_NAME_UUID) {
        rc = os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

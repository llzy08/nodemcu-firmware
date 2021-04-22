#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include "lua.h"
#include "platform.h"
#include "module.h"
#include "lauxlib.h"
#include "lextra.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// #include "gatt.h"

enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    HRS_IDX_NB,
};

#define GATTS_TAG "GATTS_DEMO"
static lua_State *gL = NULL;
esp_gatt_rsp_t rsp;

#define ARRAY_LEN(a) (sizeof(a) / sizeof(a[0]))
typedef void (*fill_cb_arg_fn)(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
typedef struct
{
    const char *name;
    esp_gatts_cb_event_t event_id;
    fill_cb_arg_fn fill_cb_arg;
} event_desc_t;

static int gatt_event_idx_by_name(const event_desc_t *table, unsigned n, const char *name);
static int gatt_event_idx_by_id(const event_desc_t *table, unsigned n, esp_gatts_cb_event_t id);
static void gatt_read(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatt_write_evt(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatt_mtu_evt(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void empty_arg(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){};
static void on_event(esp_gatts_cb_event_t evt, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const event_desc_t events[] =
{
    {"reg", ESP_GATTS_REG_EVT, empty_arg},
    {"read", ESP_GATTS_READ_EVT, gatt_read},
    {"write", ESP_GATTS_WRITE_EVT, gatt_write_evt},
    {"exec_write", ESP_GATTS_EXEC_WRITE_EVT, empty_arg},
    {"mtu", ESP_GATTS_MTU_EVT, gatt_mtu_evt},
    {"conf", ESP_GATTS_CONF_EVT, empty_arg},
    {"unreg", ESP_GATTS_UNREG_EVT, empty_arg},
    {"create", ESP_GATTS_CREATE_EVT, empty_arg},
    {"add_incl_sevc", ESP_GATTS_ADD_INCL_SRVC_EVT, empty_arg},
    {"add_char", ESP_GATTS_ADD_CHAR_EVT, empty_arg},
    {"add_char_descr", ESP_GATTS_ADD_CHAR_DESCR_EVT, empty_arg},
    {"delete", ESP_GATTS_DELETE_EVT, empty_arg},
    {"start", ESP_GATTS_START_EVT, empty_arg},
    {"stop", ESP_GATTS_STOP_EVT, empty_arg},
    {"connect", ESP_GATTS_CONNECT_EVT, empty_arg},
    {"disconnect", ESP_GATTS_DISCONNECT_EVT, empty_arg},
    {"open", ESP_GATTS_OPEN_EVT, empty_arg},
    {"cancel_open", ESP_GATTS_CANCEL_OPEN_EVT, empty_arg},
    {"close", ESP_GATTS_CLOSE_EVT, empty_arg},
    {"listen", ESP_GATTS_LISTEN_EVT, empty_arg},
    {"congest", ESP_GATTS_CONGEST_EVT, empty_arg},
    {"response", ESP_GATTS_RESPONSE_EVT, empty_arg},
    {"creat_attr", ESP_GATTS_CREAT_ATTR_TAB_EVT, empty_arg},
    {"set_attr", ESP_GATTS_SET_ATTR_VAL_EVT, empty_arg},
    {"send_service_change", ESP_GATTS_SEND_SERVICE_CHANGE_EVT, empty_arg},
};

static int event_cb[ARRAY_LEN(events)];

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "ESP_GATTS_DEMO"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 256
#define PREPARE_BUF_MAX_SIZE        256
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

typedef struct {
    esp_ble_gatts_cb_param_t *param;
    esp_gatt_if_t            gatts_if;
    uint16_t                 handle;
} sige_c_t;

static sige_c_t heart_rate_handle_table[HRS_IDX_NB] = { 
     // Service Declaration
    [IDX_SVC]        = {
        .handle = NULL,
        .param = NULL,
        .gatts_if = ESP_GATT_IF_NONE
    },
    /* Characteristic Declaration */
    [IDX_CHAR_A]     = {
        .handle = NULL,
        .param = NULL,
        .gatts_if = ESP_GATT_IF_NONE
    },
    /* Characteristic Value */
    [IDX_CHAR_VAL_A] = {
        .handle = NULL,
        .param = NULL,
        .gatts_if = ESP_GATT_IF_NONE
    },
    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A]  = {
        .handle = NULL,
        .param = NULL,
        .gatts_if = ESP_GATT_IF_NONE
    },
    /* Characteristic Declaration */
    [IDX_CHAR_B]      = {
        .handle = NULL,
        .param = NULL,
        .gatts_if = ESP_GATT_IF_NONE
    },
    /* Characteristic Value */
    [IDX_CHAR_VAL_B]  = {
        .handle = NULL,
        .param = NULL,
        .gatts_if = ESP_GATT_IF_NONE
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D','E', 'M', 'O'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0xAAAA;
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xCCC0;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xCCC1;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write_notify        = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    /* Characteristic Declaration */
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};


void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            if (heart_rate_handle_table[IDX_CHAR_VAL_B].handle == param->read.handle) {
                 int idx = gatt_event_idx_by_id(events, ARRAY_LEN(events), event);
                if (idx < 0 || event_cb[idx] == LUA_NOREF) { // 蓝牙没订阅要返回一个空
                    ESP_LOGI(GATTS_TAG, "ESP_GATTS_READ_EVT DEFAULT READ");
                    rsp.attr_value.handle = param->read.handle;
                    rsp.attr_value.len = 0;
                    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                                ESP_GATT_OK, &rsp);
                } else {
                    ESP_LOGI(GATTS_TAG, "ESP_GATTS_READ_EVT");
                    heart_rate_handle_table[IDX_CHAR_VAL_B].gatts_if = gatts_if;
                    heart_rate_handle_table[IDX_CHAR_VAL_B].param = param;
                }
            }
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                if (heart_rate_handle_table[IDX_CHAR_CFG_A].handle == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        heart_rate_handle_table[IDX_CHAR_VAL_A].gatts_if = gatts_if;
                        heart_rate_handle_table[IDX_CHAR_VAL_A].param = param;
                        return on_event(ESP_GATTS_CONNECT_EVT, gatts_if, param);
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        heart_rate_handle_table[IDX_CHAR_VAL_A].gatts_if = gatts_if;
                        heart_rate_handle_table[IDX_CHAR_VAL_A].param = param;
                        return on_event(ESP_GATTS_CONNECT_EVT, gatts_if, param);
                    }
                    else if (descr_value == 0x0000){
                        heart_rate_handle_table[IDX_CHAR_VAL_A].gatts_if = ESP_GATT_IF_NONE;
                        ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                    } else {
                        ESP_LOGE(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp) {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            } else {
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT: 
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            // 链接事件特殊处理一下，只有应用订阅了通知通道，才认为连接成功
            return;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB){
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                for (size_t i = 0; i < HRS_IDX_NB ; i++)
                {
                    heart_rate_handle_table[i].handle = param->add_attr_tab.handles[i];
                }
                esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC].handle);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break; 
    }
    on_event(event, gatts_if, param);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}




static int gatt_event_idx_by_name(const event_desc_t *table, unsigned n, const char *name)
{
    for (unsigned i = 0; i < n; ++i)
        if (strcmp(table[i].name, name) == 0)
            return i;
    return -1;
}

static int gatt_event_idx_by_id(const event_desc_t *table, unsigned n, esp_gatts_cb_event_t id)
{
    for (unsigned i = 0; i < n; ++i)
        if (table[i].event_id == id)
            return i;
    return -1;
}


static void gatt_read(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    lua_pushnumber(L, param->read.conn_id);
    lua_setfield(L, -2, "conn_id");

    lua_pushnumber(L, param->read.trans_id);
    lua_setfield(L, -2, "trans_id");
}

static void gatt_write_evt(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    lua_pushnumber(L, param->write.conn_id);
    lua_setfield(L, -2, "conn_id");

    lua_pushnumber(L, param->write.trans_id);
    lua_setfield(L, -2, "trans_id");
    lua_pushlstring(L, (const char *)param->write.value, param->write.len);
    lua_setfield(L, -2, "write_value");
}

static void gatt_mtu_evt(lua_State *L, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    lua_pushnumber(L, param->mtu.mtu);
    lua_setfield(L, -2, "mtu");
}



static int gatt_start(lua_State *L)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return 0;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return 0;
    }
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return 0;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return 0;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return 0;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return 0;
    }
    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return 0;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    return 0;
}

static int gatt_stop(lua_State *L)
{
    return 0;
}

static void on_event(esp_gatts_cb_event_t evt, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    int idx = gatt_event_idx_by_id(events, ARRAY_LEN(events), evt);
    if (idx < 0 || event_cb[idx] == LUA_NOREF || !gL) 
        return;
    
    int top = lua_gettop(gL);
    lua_rawgeti(gL, LUA_REGISTRYINDEX, event_cb[idx]);
    lua_pushstring(gL, events[idx].name);
    lua_createtable(gL, 0, 5);
    events[idx].fill_cb_arg(gL, gatts_if, param);
    lua_call(gL, 2, 0);
    lua_settop(gL, top);
}

// test pass
static int gatt_on(lua_State *L)
{
    const char *event_name = luaL_checkstring(L, 1);
    ESP_LOGE(GATTS_TAG, "gatt on name: %s", event_name);
    if (!lua_isnoneornil(L, 2))
        luaL_checkanyfunction(L, 2);
    lua_settop(L, 2);

    int idx = gatt_event_idx_by_name(events, ARRAY_LEN(events), event_name);
    ESP_LOGE(GATTS_TAG, "gatt on idx: %d", idx);
    if (idx < 0)
        return luaL_error(L, "unknown event: %s", event_name);

    luaL_unref(L, LUA_REGISTRYINDEX, event_cb[idx]);
    event_cb[idx] = lua_isnoneornil(L, 2) ? LUA_NOREF : luaL_ref(L, LUA_REGISTRYINDEX);

    gL = L;

    return 0;
}

static int gatt_notify (lua_State *L) {
    if (heart_rate_handle_table[IDX_CHAR_VAL_A].gatts_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(GATTS_TAG, "gatts notfiy error, not subscribe");
        return 0;
    }
    int total = lua_gettop( L ), s;
    const char* buf;
    size_t datalen = 0;
    for( s = 1; s <= total; s ++ )
    {
        buf = luaL_checklstring(L, s, &datalen);
        ESP_LOGE(GATTS_TAG, "gatt on idx: %s , %d", buf, datalen);

        esp_ble_gatts_send_indicate(
            heart_rate_handle_table[IDX_CHAR_VAL_A].gatts_if,
            heart_rate_handle_table[IDX_CHAR_VAL_A].param->write.conn_id,
            heart_rate_handle_table[IDX_CHAR_VAL_A].handle,
            datalen, buf, false);
    }
    return 0;
}

static int gatt_response (lua_State *L) {
    if (heart_rate_handle_table[IDX_CHAR_VAL_B].gatts_if == ESP_GATT_IF_NONE) {
        ESP_LOGE(GATTS_TAG, "gatts response error, not call read");
        return 0;
    }
    uint16_t conn_id = lua_tonumber(L, 1);
    uint32_t trans_id = lua_tonumber(L, 2);
    size_t len = 0;
    const char* buf = luaL_checklstring(L, 3, &len);
    rsp.attr_value.handle = heart_rate_handle_table[IDX_CHAR_VAL_B].handle;
    rsp.attr_value.len = len;
    memcpy(rsp.attr_value.value, buf, len);
    esp_err_t ret = esp_ble_gatts_send_response(heart_rate_handle_table[IDX_CHAR_VAL_B].gatts_if, conn_id, trans_id,
                                ESP_GATT_OK, &rsp);
                            
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts response error, error code = %x", ret);
    }                           
    return 0;
}

static int gatt_setBleName (lua_State *L) {
    const char* buf = luaL_checklstring(L, 1, NULL);
    ESP_LOGE(GATTS_TAG, "set ble name :%s", buf);
    esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(buf);
    if (set_dev_name_ret)
    {
        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
    }
    return 0;
}

static int gatt_init(lua_State *L)
{
    for (unsigned i = 0; i < ARRAY_LEN(event_cb); ++i)
    {
        event_cb[i] = LUA_NOREF;
    }
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    return 0;
}

// Module function map
LROT_BEGIN(gatt)
LROT_FUNCENTRY(start, gatt_start)
LROT_FUNCENTRY(stop, gatt_stop)
LROT_FUNCENTRY(on, gatt_on)
LROT_FUNCENTRY(notfiy, gatt_notify)
LROT_FUNCENTRY(response, gatt_response)
LROT_FUNCENTRY(setblename, gatt_setBleName)
LROT_END(gatt, NULL, 0)

NODEMCU_MODULE(GATT, "gatt", gatt, gatt_init);
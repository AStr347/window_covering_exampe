#ifndef ZIGBEE_WINDOW_COVERING_H
#define ZIGBEE_WINDOW_COVERING_H
/* base includes */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
/* freertos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
/* esp includes */
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "nvs_flash.h"
#include "esp_partition.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef bool b8;

#define TAG (__func__)

#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"

// light sleep blocker
typedef struct ia_speedstep_ctx_s {
	volatile u64 consumers;
	volatile u8 consumers_cnt;
	esp_pm_lock_handle_t handle;
} ia_speedstep_ctx_t;
#define CPU_MIN_FREQ_MHZ 	(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ)//(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ / 4)

#include "esp_zigbee.h"
#include "ezbee/zha.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       	(false)	/* enable the install code policy for security */
#define ED_AGING_TIMEOUT                	(EZB_NWK_ED_TIMEOUT_64MIN)
#define ED_KEEP_ALIVE                   	(2000)
#define EZB_PRIMARY_CHANNEL_MASK     		((1U << 13))  /* Zigbee primary channel mask use in the example */
#define EZB_SECONDARY_CHANNEL_MASK 			(0x07FFF800U)
#define ESP_ZIGBEE_STORAGE_PARTITION_NAME 	"zb_storage"

#define HA_ESP_WINDOW_COVERING_ENDPOINT 	12

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x0b""Istok-Audio"
#define ESP_MODEL_IDENTIFIER "\x06""IA.WOZ"

#define ZIGBEE_WINDOW_COVERING
#define HA_ESP_USE_ENDPOINT 	(HA_ESP_WINDOW_COVERING_ENDPOINT)
#define HA_ESP_USE_DEVICE_ID 	(EZB_ZHA_WINDOW_COVERING_CONTROLLER_DEVICE_ID)
#define HA_ESP_USE_CLUSTER_ID 	(EZB_ZCL_CLUSTER_ID_WINDOW_COVERING)

#define WINDOW_COVERING_TRY_CUSTOM_HANDLERS
//#define WINDOW_COVERING_PROCESS_LOG_REGISTERS

#define EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_DEFAULT_VALUE (EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_ONLINE | EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_OPERATIONAL)

/**
 * stora BASIC, IDENTIFY, GROUPS, SCENES clusters data
 */
typedef struct mandatory_clusters_data_s {
	/* mandatory clusters */
	ezb_zcl_cluster_desc_t basic_cluster;
	/* basic_cluster */
	u8 basic_zcl_version;
	u8 basic_power_source;

	ezb_zcl_cluster_desc_t identify_cluster;
	/* identify_cluster */
	u16 identify_time;

	ezb_zcl_cluster_desc_t groups_cluster;
	/* groups_cluster */
	u8 groups_name_support;

	ezb_zcl_cluster_desc_t scenes_cluster;
	/* scenes_cluster */
	u8 scenes_scene_count;
	u8 scenes_current_scene;
	u16 scenes_current_group;
	bool scenes_scene_valid;
	u8 scenes_name_support;
} mandatory_clusters_data_t;

// store WINDOW_COVERING cluster data
typedef struct window_covering_cluster_data_s {
	/* window by window cover cluster */
	ezb_zcl_cluster_desc_t window_covering_cluster;
	/* window_covering_cluster */
	ezb_zcl_window_covering_server_window_covering_type_t wc_type;
	u16 limit_lift;
	u16 limit_tilt;
	u16 cur_pos_lift;
	u16 cur_pos_tilt;
	u16 number_of_actuations_lift;
	u16 number_of_actuations_tilt;
	u8 config_status;
	u8 cur_pos_lift_percent;
	u8 cur_pos_tilt_percent;
} window_covering_cluster_data_t;

// zigbee top level states
typedef enum zb_state_e {
	IZBS_NONE = 0,
	IZBS_INITED,
	IZBS_STEERING,
	IZBS_CONNECTED,
} zb_state_t;

static inline
const char* ezb_zcl_core_action_callback_id_to_string(const ezb_zcl_core_action_callback_id_t id){
	switch(id){
		case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID: { return "EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID"; } break;
		case EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID: { return "EZB_ZCL_CORE_READ_ATTR_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_WRITE_ATTR_RSP_CB_ID: { return "EZB_ZCL_CORE_WRITE_ATTR_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_CONFIG_REPORT_RSP_CB_ID: { return "EZB_ZCL_CORE_CONFIG_REPORT_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_READ_REPORT_CONFIG_RSP_CB_ID: { return "EZB_ZCL_CORE_READ_REPORT_CONFIG_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_REPORT_ATTR_CB_ID: { return "EZB_ZCL_CORE_REPORT_ATTR_CB_ID"; } break;
		case EZB_ZCL_CORE_DISC_ATTR_RSP_CB_ID: { return "EZB_ZCL_CORE_DISC_ATTR_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_DISC_CMD_RSP_CB_ID: { return "EZB_ZCL_CORE_DISC_CMD_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: { return "EZB_ZCL_CORE_DEFAULT_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_MANUF_SPEC_CMD_CB_ID: { return "EZB_ZCL_CORE_MANUF_SPEC_CMD_CB_ID"; } break;
		case EZB_ZCL_CORE_IDENTIFY_EFFECT_CB_ID: { return "EZB_ZCL_CORE_IDENTIFY_EFFECT_CB_ID"; } break;
		case EZB_ZCL_CORE_BASIC_RESET_TO_FACTORY_DEFAULT_CB_ID: { return "EZB_ZCL_CORE_BASIC_RESET_TO_FACTORY_DEFAULT_CB_ID"; } break;
		case EZB_ZCL_CORE_ON_OFF_OFF_WITH_EFFECT_CB_ID: { return "EZB_ZCL_CORE_ON_OFF_OFF_WITH_EFFECT_CB_ID"; } break;
		case EZB_ZCL_CORE_GROUPS_ADD_GROUP_RSP_CB_ID: { return "EZB_ZCL_CORE_GROUPS_ADD_GROUP_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_GROUPS_VIEW_GROUP_RSP_CB_ID: { return "EZB_ZCL_CORE_GROUPS_VIEW_GROUP_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_GROUPS_GET_GROUP_MEMBERSHIP_RSP_CB_ID: { return "EZB_ZCL_CORE_GROUPS_GET_GROUP_MEMBERSHIP_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_GROUPS_REMOVE_GROUP_RSP_CB_ID: { return "EZB_ZCL_CORE_GROUPS_REMOVE_GROUP_RSP_CB_ID"; } break;case EZB_ZCL_CORE_SCENES_OPERATE_SCENE_RSP_CB_ID: { return "EZB_ZCL_CORE_SCENES_OPERATE_SCENE_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_SCENES_VIEW_SCENE_RSP_CB_ID: { return "EZB_ZCL_CORE_SCENES_VIEW_SCENE_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_SCENES_GET_SCENE_MEMBERSHIP_RSP_CB_ID: { return "EZB_ZCL_CORE_SCENES_GET_SCENE_MEMBERSHIP_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_SCENES_STORE_SCENE_CB_ID: { return "EZB_ZCL_CORE_SCENES_STORE_SCENE_CB_ID"; } break;
		case EZB_ZCL_CORE_SCENES_RECALL_SCENE_CB_ID: { return "EZB_ZCL_CORE_SCENES_RECALL_SCENE_CB_ID"; } break;
		case EZB_ZCL_CORE_DOOR_LOCK_LOCK_DOOR_CB_ID: { return "EZB_ZCL_CORE_DOOR_LOCK_LOCK_DOOR_CB_ID"; } break;
		case EZB_ZCL_CORE_DOOR_LOCK_UNLOCK_DOOR_CB_ID: { return "EZB_ZCL_CORE_DOOR_LOCK_UNLOCK_DOOR_CB_ID"; } break;
		case EZB_ZCL_CORE_DOOR_LOCK_LOCK_DOOR_RSP_CB_ID: { return "EZB_ZCL_CORE_DOOR_LOCK_LOCK_DOOR_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_DOOR_LOCK_UNLOCK_DOOR_RSP_CB_ID: { return "EZB_ZCL_CORE_DOOR_LOCK_UNLOCK_DOOR_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID: { return "EZB_ZCL_CORE_WINDOW_COVERING_MOVEMENT_CB_ID"; } break;
		case EZB_ZCL_CORE_COLOR_CONTROL_COLOR_MODE_CHANGE_CB_ID: { return "EZB_ZCL_CORE_COLOR_CONTROL_COLOR_MODE_CHANGE_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_ARM_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_ARM_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_BYPASS_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_BYPASS_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_EMERGENCY_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_EMERGENCY_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_FIRE_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_FIRE_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_PANIC_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_PANIC_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_GET_PANEL_STATUS_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_GET_PANEL_STATUS_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_GET_ZONE_STATUS_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_GET_ZONE_STATUS_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_ARM_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_ARM_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_GET_ZONE_ID_MAP_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_GET_ZONE_ID_MAP_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_GET_ZONE_INFO_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_GET_ZONE_INFO_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_ZONE_STATUS_CHANGED_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_ZONE_STATUS_CHANGED_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_PANEL_STATUS_CHANGED_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_PANEL_STATUS_CHANGED_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_GET_PANEL_STATUS_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_GET_PANEL_STATUS_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_SET_BYPASSED_ZONE_LIST_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_SET_BYPASSED_ZONE_LIST_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_BYPASS_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_BYPASS_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ACE_GET_ZONE_STATUS_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ACE_GET_ZONE_STATUS_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_WD_START_WARNING_CB_ID: { return "EZB_ZCL_CORE_IAS_WD_START_WARNING_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_WD_SQUAWK_CB_ID: { return "EZB_ZCL_CORE_IAS_WD_SQUAWK_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ZONE_INIT_TEST_MODE_CB_ID: { return "EZB_ZCL_CORE_IAS_ZONE_INIT_TEST_MODE_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ZONE_INIT_NORMAL_MODE_CB_ID: { return "EZB_ZCL_CORE_IAS_ZONE_INIT_NORMAL_MODE_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ZONE_ENROLL_CB_ID: { return "EZB_ZCL_CORE_IAS_ZONE_ENROLL_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ZONE_ENROLL_RSP_CB_ID: { return "EZB_ZCL_CORE_IAS_ZONE_ENROLL_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIF_CB_ID: { return "EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIF_CB_ID"; } break;
		case EZB_ZCL_CORE_ALARMS_ALARM_CB_ID: { return "EZB_ZCL_CORE_ALARMS_ALARM_CB_ID"; } break;
		case EZB_ZCL_CORE_ALARMS_GET_ALARM_RSP_CB_ID: { return "EZB_ZCL_CORE_ALARMS_GET_ALARM_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_ALARMS_RESET_ALARM_CB_ID: { return "EZB_ZCL_CORE_ALARMS_RESET_ALARM_CB_ID"; } break;
		case EZB_ZCL_CORE_ALARMS_RESET_ALL_ALARMS_CB_ID: { return "EZB_ZCL_CORE_ALARMS_RESET_ALL_ALARMS_CB_ID"; } break;
		case EZB_ZCL_CORE_THERMOSTAT_SETPOINT_CB_ID: { return "EZB_ZCL_CORE_THERMOSTAT_SETPOINT_CB_ID"; } break;
		case EZB_ZCL_CORE_THERMOSTAT_SET_WEEKLY_SCHEDULE_CB_ID: { return "EZB_ZCL_CORE_THERMOSTAT_SET_WEEKLY_SCHEDULE_CB_ID"; } break;
		case EZB_ZCL_CORE_THERMOSTAT_GET_WEEKLY_SCHEDULE_RSP_CB_ID: { return "EZB_ZCL_CORE_THERMOSTAT_GET_WEEKLY_SCHEDULE_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_OTA_UPGRADE_CLIENT_PROGRESS_CB_ID: { return "EZB_ZCL_CORE_OTA_UPGRADE_CLIENT_PROGRESS_CB_ID"; } break;
		case EZB_ZCL_CORE_OTA_UPGRADE_QUERY_NEXT_IMAGE_RSP_CB_ID: { return "EZB_ZCL_CORE_OTA_UPGRADE_QUERY_NEXT_IMAGE_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_OTA_UPGRADE_SERVER_PROGRESS_CB_ID: { return "EZB_ZCL_CORE_OTA_UPGRADE_SERVER_PROGRESS_CB_ID"; } break;
		case EZB_ZCL_CORE_POLL_CONTROL_CHECK_IN_CB_ID: { return "EZB_ZCL_CORE_POLL_CONTROL_CHECK_IN_CB_ID"; } break;
		case EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_PROF_INFO_CB_ID: { return "EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_PROF_INFO_CB_ID"; } break;
		case EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_MEAS_PROF_CB_ID: { return "EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_MEAS_PROF_CB_ID"; } break;
		case EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_PROF_INFO_RSP_CB_ID: { return "EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_PROF_INFO_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_MEAS_PROF_RSP_CB_ID: { return "EZB_ZCL_CORE_ELECTRICAL_MEASUREMENT_GET_MEAS_PROF_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_GET_PROFILE_CB_ID: { return "EZB_ZCL_CORE_METERING_GET_PROFILE_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_GET_PROFILE_RSP_CB_ID: { return "EZB_ZCL_CORE_METERING_GET_PROFILE_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_REQUEST_FAST_POLL_MODE_CB_ID: { return "EZB_ZCL_CORE_METERING_REQUEST_FAST_POLL_MODE_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_REQUEST_FAST_POLL_MODE_RSP_CB_ID: { return "EZB_ZCL_CORE_METERING_REQUEST_FAST_POLL_MODE_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_GET_SNAPSHOT_CB_ID: { return "EZB_ZCL_CORE_METERING_GET_SNAPSHOT_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_PUBLISH_SNAPSHOT_CB_ID: { return "EZB_ZCL_CORE_METERING_PUBLISH_SNAPSHOT_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_GET_SAMPLED_DATA_CB_ID: { return "EZB_ZCL_CORE_METERING_GET_SAMPLED_DATA_CB_ID"; } break;
		case EZB_ZCL_CORE_METERING_GET_SAMPLED_DATA_RSP_CB_ID: { return "EZB_ZCL_CORE_METERING_GET_SAMPLED_DATA_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_PRICE_GET_CURRENT_PRICE_CB_ID: { return "EZB_ZCL_CORE_PRICE_GET_CURRENT_PRICE_CB_ID"; } break;
		case EZB_ZCL_CORE_PRICE_GET_SCHEDULED_PRICES_CB_ID: { return "EZB_ZCL_CORE_PRICE_GET_SCHEDULED_PRICES_CB_ID"; } break;
		case EZB_ZCL_CORE_PRICE_GET_TIER_LABELS_CB_ID: { return "EZB_ZCL_CORE_PRICE_GET_TIER_LABELS_CB_ID"; } break;
		case EZB_ZCL_CORE_PRICE_PRICE_ACK_CB_ID: { return "EZB_ZCL_CORE_PRICE_PRICE_ACK_CB_ID"; } break;
		case EZB_ZCL_CORE_PRICE_PUBLISH_PRICE_CB_ID: { return "EZB_ZCL_CORE_PRICE_PUBLISH_PRICE_CB_ID"; } break;
		case EZB_ZCL_CORE_PRICE_PUBLISH_TIER_LABELS_CB_ID: { return "EZB_ZCL_CORE_PRICE_PUBLISH_TIER_LABELS_CB_ID"; } break;
		case EZB_ZCL_CORE_TOUCHLINK_EP_INFO_CB_ID: { return "EZB_ZCL_CORE_TOUCHLINK_EP_INFO_CB_ID"; } break;
		case EZB_ZCL_CORE_TOUCHLINK_GET_GROUP_IDS_RSP_CB_ID: { return "EZB_ZCL_CORE_TOUCHLINK_GET_GROUP_IDS_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_TOUCHLINK_GET_ENDPOINT_LIST_RSP_CB_ID: { return "EZB_ZCL_CORE_TOUCHLINK_GET_ENDPOINT_LIST_RSP_CB_ID"; } break;
		case EZB_ZCL_CORE_CB_ID_END: { return "EZB_ZCL_CORE_CB_ID_END"; } break;
	}
	return "unknown";
};

static inline
const char * ezb_bdb_comm_status_to_string(const ezb_bdb_comm_status_t err_status){
	switch (err_status){
		case EZB_BDB_STATUS_SUCCESS: { return "EZB_BDB_STATUS_SUCCESS"; } break;
		case EZB_BDB_STATUS_IN_PROGRESS: { return "EZB_BDB_STATUS_IN_PROGRESS"; } break;
		case EZB_BDB_STATUS_NOT_AA_CAPABLE: { return "EZB_BDB_STATUS_NOT_AA_CAPABLE"; } break;
		case EZB_BDB_STATUS_NO_NETWORK: { return "EZB_BDB_STATUS_NO_NETWORK"; } break;
		case EZB_BDB_STATUS_TARGET_FAILURE: { return "EZB_BDB_STATUS_TARGET_FAILURE"; } break;
		case EZB_BDB_STATUS_FORMATION_FAILURE: { return "EZB_BDB_STATUS_FORMATION_FAILURE"; } break;
		case EZB_BDB_STATUS_NO_IDENTIFY_QUERY_RESPONSE: { return "EZB_BDB_STATUS_NO_IDENTIFY_QUERY_RESPONSE"; } break;
		case EZB_BDB_STATUS_BINDING_TABLE_FULL: { return "EZB_BDB_STATUS_BINDING_TABLE_FULL"; } break;
		case EZB_BDB_STATUS_NO_SCAN_RESPONSE: { return "EZB_BDB_STATUS_NO_SCAN_RESPONSE"; } break;
		case EZB_BDB_STATUS_NOT_PERMITTED: { return "EZB_BDB_STATUS_NOT_PERMITTED"; } break;
		case EZB_BDB_STATUS_TCLK_EX_FAILURE: { return "EZB_BDB_STATUS_TCLK_EX_FAILURE"; } break;
		case EZB_BDB_STATUS_NOT_ON_A_NETWORK: { return "EZB_BDB_STATUS_NOT_ON_A_NETWORK"; } break;
		case EZB_BDB_STATUS_ON_A_NETWORK: { return "EZB_BDB_STATUS_ON_A_NETWORK"; } break;
		case EZB_BDB_STATUS_CANCELLED: { return "EZB_BDB_STATUS_CANCELLED"; } break;
		case EZB_BDB_STATUS_DEV_ANNCE_SEND_FAILURE: { return "EZB_BDB_STATUS_DEV_ANNCE_SEND_FAILURE"; } break;
	}
	return "unknown";
}

static inline
const char * zb_state_to_string(const zb_state_t state){
	static const
	char * const zb_state_names[] = {
		[IZBS_NONE] = "IZBS_NONE",
		[IZBS_INITED] = "IZBS_INITED",
		[IZBS_STEERING] = "IZBS_STEERING",
		[IZBS_CONNECTED] = "IZBS_CONNECTED",
	};
	return zb_state_names[state];
}


#endif//ZIGBEE_WINDOW_COVERING_H
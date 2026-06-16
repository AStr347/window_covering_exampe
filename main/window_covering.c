#include "window_covering.h"

ia_speedstep_ctx_t speedstep;
mandatory_clusters_data_t mandatory_clusters_data;
window_covering_cluster_data_t window_covering_cluster_data;
zb_state_t zb_state = IZBS_NONE;
u64 zb_consumer_mask = 0;

//===============================================================
//======================= Power managment =======================
//===============================================================

esp_err_t speedstep_init(void){
	/* power managment */
	ESP_LOGW(TAG, "initialize Power managment");
	speedstep.consumers = 0;
	speedstep.consumers_cnt = 0;
	const esp_err_t lock_status = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "speedstep", &speedstep.handle);
	if(ESP_OK != lock_status){
		ESP_LOGE(TAG, "esp_pm_lock_create failed");
		return ESP_FAIL;
	}

	esp_pm_config_t pm_config = {
		.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
		.min_freq_mhz = CPU_MIN_FREQ_MHZ,
		.light_sleep_enable = true,
	};
	esp_err_t configure_res = esp_pm_configure(&pm_config);
	if(ESP_OK != configure_res){
		ESP_LOGE(TAG, "esp_pm_configure failed");
		return ESP_FAIL;
	}
	return ESP_OK;
}

/**
 * register consumer
 */
u64 speedstep_register(void){
	const u64 result = (1 << speedstep.consumers_cnt);
	speedstep.consumers_cnt++;
	return result;
}

/**
 * @return true - no consumers now, false - esp32 can't sleep now
 */
b8 speedstep_can_sleep(void){
	return (0 == speedstep.consumers);
}

void speedstep_sleep_off(const u64 consumer_mask){
	if(0 == speedstep.consumers){
		esp_pm_lock_acquire(speedstep.handle);
		ESP_LOGE(TAG, "esp_pm_lock_acquire");
	}
	speedstep.consumers |= consumer_mask;
}

void speedstep_sleep_on(const u64 consumer_mask){
	speedstep.consumers &= (~consumer_mask);
	if(0 == speedstep.consumers){
		ESP_LOGE(TAG, "esp_pm_lock_release");
		esp_pm_lock_release(speedstep.handle);
	}
}

#if defined(WINDOW_COVERING_TRY_CUSTOM_HANDLERS)

//===============================================================
//================= Window covering handlers ====================
//===============================================================

static
ezb_zcl_status_t window_covering_check(const uint16_t attr_id, const uint8_t ep_id, void * const value){
	ESP_LOGW(TAG, "attr_id:%04x ep_id:%02x value:%08x", attr_id, ep_id, (u32)value);
	return EZB_ZCL_STATUS_SUCCESS;
}

static
void window_covering_write(const uint8_t ep_id, const uint16_t attr_id, void * const new_value, const uint16_t manuf_code){
	ESP_LOGW(TAG, "attr_id:%04x ep_id:%02x value:%08x manuf_code:%04x", attr_id, ep_id, (u32)new_value, manuf_code);
}

static
uint8_t window_covering_disc(const bool is_recv, const uint8_t **list){
	ESP_LOGW(TAG, "is_recv:%d");
	static const u8 support_cmd[] = {
		EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID,
		EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID,
	};
	*list = support_cmd;
	return sizeof(support_cmd);
}

static
ezb_zcl_status_t window_covering_process(const ezb_zcl_cmd_hdr_t *header, const uint8_t * payload, uint16_t payload_length)
{
#if defined(WINDOW_COVERING_PROCESS_LOG_REGISTERS)
	{
		register u32 a0_value __asm__("a0");
		register u32 a1_value __asm__("a1");
		register u32 a2_value __asm__("a2");
		register u32 a3_value __asm__("a3");
		register u32 a4_value __asm__("a4");
		register u32 a5_value __asm__("a5");
		register u32 a6_value __asm__("a6");
		register u32 a7_value __asm__("a7");
		ESP_LOGW(TAG, "a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x a6:%08x a7:%08x", a0_value, a1_value, a2_value, a3_value, a4_value, a5_value, a6_value, a7_value);
	}
#endif

	static const
	char * const cmd_names[] = {
		[EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN_ID",
		[EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE_ID",
		[EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID",
		[EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_VALUE_ID",
		[EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID",
		[EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_VALUE_ID",
		[EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE_ID] = "EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE_ID",
	};

	
	const ezb_zcl_window_covering_server_cmd_id_t com = (ezb_zcl_window_covering_server_cmd_id_t)header->cmd_id;

	const char * com_name = "unknown";
	if(com <= EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_TILT_PERCENTAGE_ID){
		com_name = cmd_names[com];
	}
	ESP_LOGI(TAG, "'%s'(%x)", com_name, com);
	ESP_LOGW("\t", "dst_ep:%02x cluster_id:%04x cmd_id:%02x payload_length:%04d payload_ptr:%08x", header->dst_ep, header->cluster_id, header->cmd_id, payload_length, (u32)payload);

	ezb_zcl_status_t result = EZB_ZCL_STATUS_UNSUP_CMD;
	switch (com) {
		case EZB_ZCL_CMD_WINDOW_COVERING_STOP_ID: {
#warning "TODO: stop motor here"
			result = EZB_ZCL_STATUS_SUCCESS;
		} break;
		case EZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE_ID: {
#warning "TODO: start move motor to desired position here"
			result = EZB_ZCL_STATUS_SUCCESS;
		} break;
		default:{
			/* nothing */
		} break;
	}
	return result;
}
#endif//WINDOW_COVERING_TRY_CUSTOM_HANDLERS

/**
 * cover esp_zb_zcl_set_attribute_val call
 * @param __ATTR_ID__ - [in] window covering attribute ID EZB_ZCL_ATTR_WINDOW_COVERING_{NAME}
 * @param __VAL__ - [in] value to write
 * 
 * @return nothing, can log
 */
#define SET_ATTR(__ATTR_ID__, __VAL__)\
{\
	u32 value = __VAL__;\
	const ezb_zcl_status_t res = ezb_zcl_set_attr_value(HA_ESP_USE_ENDPOINT,\
																HA_ESP_USE_CLUSTER_ID,\
																EZB_ZCL_CLUSTER_SERVER,\
																__ATTR_ID__,\
																EZB_ZCL_STD_MANUF_CODE,\
																&value,\
																false);\
	if(EZB_ZCL_STATUS_SUCCESS != res){\
		ESP_LOGE(TAG, "err: %x attr_id: %x value: %d", res, __ATTR_ID__, __VAL__);\
	}\
}

/**
 * update pos and percent values in Zigbee cluster
 * host should can get actual values
 * 
 * @param percent - [in] mototr position in percent
 */
void window_covering_update_params(const u8 percent){
	esp_zigbee_lock_acquire(portMAX_DELAY);
	SET_ATTR(EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, percent);
	SET_ATTR(EZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID, EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_DEFAULT_VALUE);
	esp_zigbee_lock_release();
}

//===============================================================
//=================== Find/Bind handlers ========================
//===============================================================

/**
 * default bind result callback handler
 */
static
void esp_zigbee_bind_result(const ezb_zdp_bind_req_result_t * const result, void * const user_ctx){
    assert(result);
    if (result->error != EZB_ERR_NONE) {
		ESP_LOGE(TAG, "Bind Failed error (0x%04x)", result->error);
		return;
	}
	ezb_zdp_bind_rsp_field_t * rsp = result->rsp;
	assert(rsp);
	ESP_LOGW(TAG, "error:%04x rsp.status:%02x",
		result->error,
		rsp->status
	);
	const u8 status = rsp->status;
	if (EZB_ZDP_STATUS_SUCCESS != status) {
		ESP_LOGE(TAG, "Bind Failed status (0x%02x)", status);
		return;
	}
	ESP_LOGI(TAG, "Bound HA device successfully");
}

static
ezb_err_t window_covering_bind(const ezb_shortaddr_t dst_short_addr, const u8 match, const u8 index) {
    ezb_zdo_bind_req_t bind_req = {
        .dst_nwk_addr = ezb_get_short_address(),
        .field = {
			.src_ep        	= HA_ESP_USE_ENDPOINT,
			.cluster_id    	= HA_ESP_USE_CLUSTER_ID,
			.dst_addr_mode 	= EZB_ADDR_MODE_EXT,
			.dst_ep        	= match,
		},
        .cb = esp_zigbee_bind_result,
        .user_ctx = NULL,
    };
	ezb_nwk_get_extended_address(&bind_req.field.src_addr);
	const ezb_err_t get_res = ezb_address_extended_by_short(dst_short_addr, &bind_req.field.dst_addr.extended_addr);
	if(EZB_ERR_NONE != get_res){
		ESP_LOGE(TAG, "Failed ezb_address_extended_by_short addr:0x%04x err:0x%04x", dst_short_addr, get_res);
		return get_res;
	}
    const ezb_err_t ret = ezb_zdo_bind_req(&bind_req);
    if (EZB_ERR_NONE != ret) {
		ESP_LOGE(TAG, "Failed ezb_zdo_bind_req addr:0x%04x err:0x%04x", dst_short_addr, ret);
		return ret;
    }
	ESP_LOGI(TAG, "Binded addr:0x%04x", dst_short_addr);
    return ret;
}

/**
 * default find result callback handler
 */
static
void esp_zigbee_find_cb(const ezb_zdo_match_desc_req_result_t *result, void *user_ctx) {
    assert(result);
    if (result->error != EZB_ERR_NONE) {
		ESP_LOGE(TAG, "Failed error:0x%04x", result->error);
		return;
	}
	ezb_zdp_match_desc_rsp_field_t * rsp = result->rsp;
	assert(rsp);
	ESP_LOGW(TAG, "error:%04x rsp.status:%02x rsp.nwk_addr:%04x rsp.match_len:%02x",
		result->error,
		rsp->status,
		rsp->nwk_addr_of_interest,
		rsp->match_length
	);
	const u8 status = rsp->status;
	if(EZB_ZDP_STATUS_SUCCESS != status){
		ESP_LOGE(TAG, "Failed Find status:0x%02x", status);
		return;
	}
	const u8 match_len = rsp->match_length;
	const b8 have_match_list = NULL != rsp->match_list;
	if (0 == match_len || false == have_match_list) {
		ESP_LOGE(TAG, "have not Matches");
		return;
	}
	const ezb_shortaddr_t short_dst_addr = rsp->nwk_addr_of_interest;
	for (size_t i = 0; i < match_len; i++) {
		window_covering_bind(short_dst_addr, rsp->match_list[i], i);
	}
}

/**
 * Find func for STERRING complete
 */
static
ezb_err_t esp_zigbee_find(void) {
    static const
	ezb_zcl_cluster_id_t cluster_list[] = {
		HA_ESP_USE_CLUSTER_ID,
	};
    const ezb_zdo_match_desc_req_t req = {
        .dst_nwk_addr = 0xFFFD,
        .field = {
			.nwk_addr_of_interest = 0xFFFD, // may be our short addr???
			.profile_id           = EZB_AF_HA_PROFILE_ID,
			.num_in_clusters      = (sizeof(cluster_list) / sizeof(ezb_zcl_cluster_id_t)),
			.num_out_clusters     = 0,
			.cluster_list         = (u16*)cluster_list,
		},
        .cb = esp_zigbee_find_cb,
        .user_ctx = NULL,
    };
    const ezb_err_t ret = ezb_zdo_match_desc_req(&req);
    if (ret != EZB_ERR_NONE) {
        ESP_LOGE(TAG, "Failed ezb_zdo_match_desc_req error:0x%04x", ret);
    }
    return ret;
}

//===============================================================
//===================== Zigbee helpers ==========================
//===============================================================

static
b8 esp_zigbee_is_connected(void){
	ezb_extpanid_t extended_pan_id;
	ezb_get_extended_panid(&extended_pan_id);
	const ezb_panid_t pan_id = ezb_get_panid();
	const u8 cur_ch = ezb_get_current_channel();
	const ezb_shortaddr_t short_addr = ezb_get_short_address();
	ESP_LOGI("\t", 	"network data (Extended PAN ID: %016llX,"
					"PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
					extended_pan_id.u64,
					pan_id, cur_ch, short_addr);
	const b8 is_pan_id = (0xffff != pan_id);
	// const b8 is_cur_ch = (0 != cur_ch);
	// const b8 is_short_addr = (0 != short_addr);
	return (is_pan_id);
}

static
void bdb_commissioning_saved(const ezb_bdb_comm_mode_t bdb_comm){
	esp_zigbee_lock_acquire(portMAX_DELAY);
	ezb_bdb_start_top_level_commissioning(bdb_comm);
	esp_zigbee_lock_release();
}

//===============================================================
//===================== Zigbee top level ========================
//===============================================================

static
void zb_action_handler(const ezb_zcl_core_action_callback_id_t callback_id, void *message) {
	ESP_LOGW(TAG, "'%s'(0x%x)", ezb_zcl_core_action_callback_id_to_string(callback_id), callback_id);
}

static
bool esp_zigbee_app_signal_handler(const ezb_app_signal_t * signal_struct) {
	#define LOG_EZB_SINGAL() ESP_LOGW(TAG, "signal:`%s`(0x%02x) status:`%s`(0x%02x) state:`%s`(%d)", zdo_signal_name, sig_type, err_name, err_status, prev_state_name, zb_state)
	#define LOG_EZB_SINGAL_END() ESP_LOGW("\t", "%s -> %s", prev_state_name, zb_state_to_string(zb_state));

    const ezb_app_signal_type_t sig_type = ezb_app_signal_get_type(signal_struct);
	const ezb_bdb_comm_status_t err_status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal_struct));

	const char * const zdo_signal_name = ezb_app_signal_to_string(sig_type);
	const char * const err_name = ezb_bdb_comm_status_to_string(err_status);
	const char * const prev_state_name = zb_state_to_string(zb_state);

    switch (sig_type) {
		case EZB_ZDO_SIGNAL_SKIP_STARTUP:{
			LOG_EZB_SINGAL();
			ESP_LOGI("\t", "Initialize Zigbee stack");
			bdb_commissioning_saved(EZB_BDB_MODE_INITIALIZATION);
		} break;
		case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
		case EZB_BDB_SIGNAL_DEVICE_REBOOT:{
			LOG_EZB_SINGAL();
			if(EZB_BDB_STATUS_SUCCESS == err_status) {
				const b8 is_factory_new = ezb_bdb_is_factory_new();
				ESP_LOGI("\t", "Device started up in %d factory-reset mode", is_factory_new);
				const b8 is_connected = esp_zigbee_is_connected();
				if(true == is_connected){
					/* example always 50% */
					window_covering_update_params(50);
					/* Pairing finalized can be sleepy */
					speedstep_sleep_on(zb_consumer_mask);
					zb_state = IZBS_CONNECTED;
				} else {
					zb_state = IZBS_INITED;
					/* Pairing start can't be sleepy */
					speedstep_sleep_off(zb_consumer_mask);
					bdb_commissioning_saved(EZB_BDB_MODE_NETWORK_STEERING);
				}
			} else {
				ESP_LOGW("\t", "failed with status: %s, retrying", err_name);
				bdb_commissioning_saved(EZB_BDB_MODE_INITIALIZATION);
				if(IZBS_CONNECTED == zb_state){
					ESP_LOGW("\t", "Zigbee leave");
				}
				zb_state = IZBS_NONE;
			}
			LOG_EZB_SINGAL_END();
		} break;
		case EZB_BDB_SIGNAL_STEERING:{
			LOG_EZB_SINGAL();
			const b8 is_connected = esp_zigbee_is_connected();
			if (EZB_BDB_STATUS_SUCCESS == err_status) {
				if(true == is_connected){
					esp_zigbee_find();
					/* example always 50% */
					window_covering_update_params(50);
					/* Pairing finalized can be sleepy */
					speedstep_sleep_on(zb_consumer_mask);
					zb_state = IZBS_CONNECTED;
				} else {
					zb_state = IZBS_STEERING;
				}
			} else {
				ESP_LOGI("\t", "Network steering was not successful (status: %s)", err_name);
				zb_state = IZBS_INITED;
				bdb_commissioning_saved(EZB_BDB_MODE_NETWORK_STEERING);
			}
			LOG_EZB_SINGAL_END();
		} break;

		case EZB_ZDO_SIGNAL_LEAVE:{
			LOG_EZB_SINGAL();
			const b8 is_connected = esp_zigbee_is_connected();
			if(true == is_connected){
				zb_state = IZBS_CONNECTED;
			} else {
				if(IZBS_CONNECTED == zb_state){
					ESP_LOGW("\t", "Zigbee leave");
				}
				zb_state = IZBS_INITED;
			}
			LOG_EZB_SINGAL_END();
		} break;

		default:{
			LOG_EZB_SINGAL();
		} break;
    }
	return true;
}

void esp_zb_task(void * arg){
	/* initialize Zigbee stack */
	ESP_LOGW(TAG, "initialize Zigbee stack");
	zb_consumer_mask = speedstep_register();

	static const
	esp_zigbee_config_t config = {
        .device_config = {
			.device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE,
			.install_code_policy = INSTALLCODE_POLICY_ENABLE,
			.zed_config = {
				.ed_timeout = ED_AGING_TIMEOUT,
				.keep_alive = ED_KEEP_ALIVE,
			},
		},
        .platform_config = {
			.storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME,
			.radio_config = {
				.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
			},
		},
    };
	ESP_ERROR_CHECK(esp_zigbee_init(&config));
	{
		/* esp_zigbee_setup_commissioning */
		ezb_aps_secur_enable_distributed_security(false);
		ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(EZB_PRIMARY_CHANNEL_MASK));
		ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(EZB_SECONDARY_CHANNEL_MASK));
		ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
		ezb_nwk_set_rx_on_when_idle(true);
	}

	static const
	ezb_af_ep_config_t zha_config = {
		.ep_id = HA_ESP_USE_ENDPOINT,
		.app_profile_id = EZB_AF_HA_PROFILE_ID,
		.app_device_id = HA_ESP_USE_DEVICE_ID,
		.app_device_version = 0,
	};
	ezb_af_ep_desc_t ep_desc = ezb_af_create_endpoint_desc(&zha_config);
	{
		/* mandatory clusters */
		ESP_LOGW(TAG, "initialize mandatory clusters");
		mandatory_clusters_data_t * mcd = &mandatory_clusters_data;
		{
			/* basic_cluster */
			ESP_LOGW("\t", "initialize basic_cluster");
			mcd->basic_cluster = ezb_zcl_basic_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
			mcd->basic_zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
			mcd->basic_power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY;

			ezb_zcl_basic_cluster_desc_add_attr(mcd->basic_cluster, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
			ezb_zcl_basic_cluster_desc_add_attr(mcd->basic_cluster, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER);
			ezb_zcl_basic_cluster_desc_add_attr(mcd->basic_cluster, EZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &mcd->basic_zcl_version);
			ezb_zcl_basic_cluster_desc_add_attr(mcd->basic_cluster, EZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &mcd->basic_power_source);
		}

		{
			/* identify_cluster */
			ESP_LOGW("\t", "initialize identify_cluster");
			mcd->identify_cluster = ezb_zcl_identify_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
			mcd->identify_time = EZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

			ezb_zcl_identify_cluster_desc_add_attr(mcd->identify_cluster, EZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &mcd->identify_time);
		}
		
		{
			/* groups_cluster */
			ESP_LOGW("\t", "initialize groups_cluster");
			mcd->groups_cluster = ezb_zcl_groups_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
			mcd->groups_name_support = EZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE;
			
			ezb_zcl_groups_cluster_desc_add_attr(mcd->groups_cluster, EZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &mcd->groups_name_support);
		}

		{
			/* scenes_cluster */
			ESP_LOGW("\t", "initialize scenes_cluster");
			mcd->scenes_cluster = ezb_zcl_scenes_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
			mcd->scenes_scene_count = EZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE;
			mcd->scenes_current_scene = EZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE;
			mcd->scenes_current_group = EZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE;
			mcd->scenes_scene_valid = EZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE;
			mcd->scenes_name_support = EZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE;
			
			ezb_zcl_scenes_cluster_desc_add_attr(mcd->scenes_cluster, EZB_ZCL_ATTR_SCENES_SCENE_COUNT_ID, &mcd->scenes_scene_count);
			ezb_zcl_scenes_cluster_desc_add_attr(mcd->scenes_cluster, EZB_ZCL_ATTR_SCENES_CURRENT_SCENE_ID, &mcd->scenes_current_scene);
			ezb_zcl_scenes_cluster_desc_add_attr(mcd->scenes_cluster, EZB_ZCL_ATTR_SCENES_CURRENT_GROUP_ID, &mcd->scenes_current_group);
			ezb_zcl_scenes_cluster_desc_add_attr(mcd->scenes_cluster, EZB_ZCL_ATTR_SCENES_SCENE_VALID_ID, &mcd->scenes_scene_valid);
			ezb_zcl_scenes_cluster_desc_add_attr(mcd->scenes_cluster, EZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &mcd->scenes_name_support);
		}

		ESP_LOGW("\t", "add mandatory clusters to endpoint");
		ezb_af_endpoint_add_cluster_desc(ep_desc, mcd->basic_cluster);
		ezb_af_endpoint_add_cluster_desc(ep_desc, mcd->identify_cluster);
		ezb_af_endpoint_add_cluster_desc(ep_desc, mcd->groups_cluster);
		ezb_af_endpoint_add_cluster_desc(ep_desc, mcd->scenes_cluster);
	}
	{
		/* window_covering_cluster */
		ESP_LOGW(TAG, "initialize window_covering_cluster");
		window_covering_cluster_data_t * wccd = &window_covering_cluster_data;
		wccd->window_covering_cluster = ezb_zcl_window_covering_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
		wccd->wc_type = EZB_ZCL_WINDOW_COVERING_WINDOW_COVERING_TYPE_ROLLERSHADE;
		wccd->limit_lift = EZB_ZCL_WINDOW_COVERING_PHYSICAL_CLOSED_LIMIT_LIFT_DEFAULT_VALUE;
		wccd->limit_tilt = EZB_ZCL_WINDOW_COVERING_PHYSICAL_CLOSED_LIMIT_TILT_DEFAULT_VALUE;
		wccd->cur_pos_lift = 0;
		wccd->cur_pos_tilt = 0;
		wccd->number_of_actuations_lift = EZB_ZCL_WINDOW_COVERING_NUMBER_OF_ACTUATIONS_LIFT_DEFAULT_VALUE;
		wccd->number_of_actuations_tilt = EZB_ZCL_WINDOW_COVERING_NUMBER_OF_ACTUATIONS_TILT_DEFAULT_VALUE;
		wccd->config_status = EZB_ZCL_WINDOW_COVERING_CONFIG_STATUS_DEFAULT_VALUE;
		wccd->cur_pos_lift_percent = 0;
		wccd->cur_pos_tilt_percent = 0;

		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID, &wccd->wc_type);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_PHYSICAL_CLOSED_LIMIT_LIFT_ID, &wccd->limit_lift);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_PHYSICAL_CLOSED_LIMIT_TILT_ID, &wccd->limit_tilt);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_ID, &wccd->cur_pos_lift);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_ID, &wccd->cur_pos_tilt);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_NUMBER_OF_ACTUATIONS_LIFT_ID, &wccd->number_of_actuations_lift);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_NUMBER_OF_ACTUATIONS_TILT_ID, &wccd->number_of_actuations_tilt);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID, &wccd->config_status);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &wccd->cur_pos_lift_percent);
		ezb_zcl_window_covering_cluster_desc_add_attr(wccd->window_covering_cluster, EZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_TILT_PERCENTAGE_ID, &wccd->cur_pos_tilt_percent);

		ESP_LOGW("\t", "add window_covering_cluster to endpoint");
		ezb_af_endpoint_add_cluster_desc(ep_desc, wccd->window_covering_cluster);
#if defined(WINDOW_COVERING_TRY_CUSTOM_HANDLERS)
			static const
			ezb_zcl_custom_cluster_handlers_t handlers = {
			.cluster_id = HA_ESP_USE_CLUSTER_ID,
			.cluster_role = EZB_ZCL_CLUSTER_SERVER,
			.check_value_cb = (ezb_zcl_custom_cluster_check_value_t)window_covering_check,
			.write_attr_cb = (ezb_zcl_custom_cluster_write_attr_t)window_covering_write,
			.cmd_disc_cb = (ezb_zcl_custom_cluster_disc_cmd_t)window_covering_disc,
			.process_cmd_cb = (ezb_zcl_custom_cluster_process_cmd_t)window_covering_process,
		};
		ezb_zcl_custom_cluster_handlers_register(&handlers);
#endif
		ezb_zcl_window_covering_cluster_server_init(HA_ESP_USE_ENDPOINT);
	}

	ESP_LOGW(TAG, "initialize Zigbee device");
	ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
	ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
    ezb_zcl_core_action_handler_register((ezb_zcl_core_action_callback_t) zb_action_handler);

	ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}


void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));
	speedstep_init();

	xTaskCreate(esp_zb_task, "Zigbee_main", 1024 * 6, NULL, 10, NULL);
}

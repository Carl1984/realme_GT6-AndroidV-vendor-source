/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <osdep.h>
#include "api/fw/wmi.h"
#include "wmi_unified_priv.h"
#include "wmi_unified_cfr_param.h"
#include "wmi_unified_cfr_api.h"

#ifdef WLAN_CFR_ENABLE
static QDF_STATUS
extract_cfr_peer_tx_event_param_tlv(wmi_unified_t wmi_handle, void *evt_buf,
				    wmi_cfr_peer_tx_event_param *peer_tx_event)
{
	int idx;
	WMI_PEER_CFR_CAPTURE_EVENTID_param_tlvs *param_buf;
	wmi_peer_cfr_capture_event_fixed_param *peer_tx_event_ev;
	wmi_peer_cfr_capture_event_phase_fixed_param *chain_phase_ev;

	param_buf = (WMI_PEER_CFR_CAPTURE_EVENTID_param_tlvs *)evt_buf;
	if (!param_buf) {
		wmi_err("Invalid cfr capture buffer");
		return QDF_STATUS_E_INVAL;
	}

	peer_tx_event_ev = param_buf->fixed_param;
	if (!peer_tx_event_ev) {
		wmi_err("peer cfr capture buffer is null");
		return QDF_STATUS_E_NULL_VALUE;
	}

	peer_tx_event->capture_method = peer_tx_event_ev->capture_method;
	peer_tx_event->vdev_id = peer_tx_event_ev->vdev_id;
	WMI_MAC_ADDR_TO_CHAR_ARRAY(&peer_tx_event_ev->mac_addr,
				   &peer_tx_event->peer_mac_addr.bytes[0]);
	peer_tx_event->primary_20mhz_chan =
		peer_tx_event_ev->chan_mhz;
	peer_tx_event->bandwidth = peer_tx_event_ev->bandwidth;
	peer_tx_event->phy_mode = peer_tx_event_ev->phy_mode;
	peer_tx_event->band_center_freq1 = peer_tx_event_ev->band_center_freq1;
	peer_tx_event->band_center_freq2 = peer_tx_event_ev->band_center_freq2;
	peer_tx_event->spatial_streams = peer_tx_event_ev->sts_count;
	peer_tx_event->correlation_info_1 =
		peer_tx_event_ev->correlation_info_1;
	peer_tx_event->correlation_info_2 =
		peer_tx_event_ev->correlation_info_2;
	peer_tx_event->status = peer_tx_event_ev->status;
	peer_tx_event->timestamp_us = peer_tx_event_ev->timestamp_us;
	peer_tx_event->counter = peer_tx_event_ev->counter;
	qdf_mem_copy(peer_tx_event->chain_rssi, peer_tx_event_ev->chain_rssi,
		     sizeof(peer_tx_event->chain_rssi));
	if (peer_tx_event_ev->cfo_measurement_valid)
		peer_tx_event->cfo_measurement =
			peer_tx_event_ev->cfo_measurement;
	else
		peer_tx_event->cfo_measurement = 0;

	peer_tx_event->rx_start_ts = peer_tx_event_ev->rx_start_ts;
	peer_tx_event->rx_ts_reset = peer_tx_event_ev->rx_ts_reset;
	peer_tx_event->mcs_rate =
		WMI_CFR_MCS_GET(peer_tx_event_ev->mcs_gi_info);
	peer_tx_event->gi_type =
		WMI_CFR_GI_TYPE_GET(peer_tx_event_ev->mcs_gi_info);

	chain_phase_ev = param_buf->phase_param;
	if (chain_phase_ev) {
		for (idx = 0; idx < WMI_HOST_MAX_CHAINS; idx++) {
			/* Due to FW's alignment rules, phase information being
			 * passed is 32-bit, out of which only 16 bits is valid.
			 * Remaining bits are all zeroed. So direct mem copy
			 * will not work as it will copy extra zeroes into host
			 * structures.
			 */
			peer_tx_event->chain_phase[idx] =
				(0xffff & chain_phase_ev->chain_phase[idx]);
			peer_tx_event->agc_gain[idx] =
				WMI_UNIFIED_AGC_GAIN_GET(chain_phase_ev, idx);
			peer_tx_event->agc_gain_tbl_index[idx] =
				WMI_UNIFIED_AGC_GAIN_TBL_IDX_GET(chain_phase_ev,
								 idx);
		}
	}

	return QDF_STATUS_SUCCESS;
}

#ifdef WLAN_ENH_CFR_ENABLE
static void populate_wmi_cfr_param(uint8_t grp_id, struct cfr_rcc_param *rcc,
				   wmi_cfr_filter_group_config *param)
{
	struct ta_ra_cfr_cfg *tgt_cfg = NULL;

	WMITLV_SET_HDR(&param->tlv_header,
		       WMITLV_TAG_STRUC_wmi_cfr_filter_group_config,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_cfr_filter_group_config));
	tgt_cfg  = &rcc->curr[grp_id];

	param->filter_group_id = grp_id;
	WMI_CFR_GROUP_TA_ADDR_VALID_SET(param->filter_set_valid_mask,
					tgt_cfg->valid_ta);
	WMI_CFR_GROUP_TA_ADDR_MASK_VALID_SET(param->filter_set_valid_mask,
					     tgt_cfg->valid_ta_mask);
	WMI_CFR_GROUP_RA_ADDR_VALID_SET(param->filter_set_valid_mask,
					tgt_cfg->valid_ra);
	WMI_CFR_GROUP_RA_ADDR_MASK_VALID_SET(param->filter_set_valid_mask,
					     tgt_cfg->valid_ra_mask);
	WMI_CFR_GROUP_BW_VALID_SET(param->filter_set_valid_mask,
				   tgt_cfg->valid_bw_mask);
	WMI_CFR_GROUP_NSS_VALID_SET(param->filter_set_valid_mask,
				    tgt_cfg->valid_nss_mask);
	WMI_CFR_GROUP_MGMT_SUBTYPE_VALID_SET(param->filter_set_valid_mask,
					     tgt_cfg->valid_mgmt_subtype);
	WMI_CFR_GROUP_CTRL_SUBTYPE_VALID_SET(param->filter_set_valid_mask,
					     tgt_cfg->valid_ctrl_subtype);
	WMI_CFR_GROUP_DATA_SUBTYPE_VALID_SET(param->filter_set_valid_mask,
					     tgt_cfg->valid_data_subtype);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(tgt_cfg->tx_addr,
				   &param->ta_addr);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(tgt_cfg->tx_addr_mask,
				   &param->ta_addr_mask);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(tgt_cfg->rx_addr,
				   &param->ra_addr);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(tgt_cfg->rx_addr_mask,
				   &param->ra_addr_mask);
	WMI_CFR_GROUP_BW_SET(param->bw_nss_filter,
			     tgt_cfg->bw);
	WMI_CFR_GROUP_NSS_SET(param->bw_nss_filter,
			      tgt_cfg->nss);
	param->mgmt_subtype_filter = tgt_cfg->mgmt_subtype_filter;
	param->ctrl_subtype_filter = tgt_cfg->ctrl_subtype_filter;
	param->data_subtype_filter = tgt_cfg->data_subtype_filter;
}

static QDF_STATUS send_cfr_rcc_cmd_tlv(wmi_unified_t wmi_handle,
				       struct cfr_rcc_param *rcc)
{
	wmi_cfr_capture_filter_cmd_fixed_param *cmd;
	wmi_cfr_filter_group_config *param;
	uint8_t *buf_ptr, grp_id;
	wmi_buf_t buf;
	uint32_t len;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct wmi_ops *ops = wmi_handle->ops;

	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE;
	len += rcc->num_grp_tlvs * sizeof(wmi_cfr_filter_group_config);
	buf = wmi_buf_alloc(wmi_handle, len);

	if (!buf) {
		wmi_err("wmi_buf_alloc failed");
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = wmi_buf_data(buf);
	cmd = (wmi_cfr_capture_filter_cmd_fixed_param *)buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_cfr_capture_filter_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_cfr_capture_filter_cmd_fixed_param));
	cmd->pdev_id = ops->convert_host_pdev_id_to_target(wmi_handle,
							   rcc->pdev_id);
	WMI_CFR_CAPTURE_INTERVAL_SET(cmd->capture_interval,
				     rcc->capture_interval);
	WMI_CFR_CAPTURE_DURATION_SET(cmd->capture_duration,
				     rcc->capture_duration);
	WMI_CFR_CAPTURE_COUNT_SET(cmd->capture_count, rcc->capture_count);
	WMI_CFR_CAPTURE_INTERVAL_MODE_SEL_SET(cmd->capture_count,
					      rcc->capture_intval_mode_sel);
	WMI_CFR_FILTER_GROUP_BITMAP_SET(cmd->filter_group_bitmap,
					rcc->filter_group_bitmap);
	WMI_CFR_UL_MU_USER_UPPER_SET(cmd->ul_mu_user_mask_upper,
				     rcc->ul_mu_user_mask_upper);
	cmd->ul_mu_user_mask_lower = rcc->ul_mu_user_mask_lower;
	WMI_CFR_FREEZE_DELAY_CNT_EN_SET(cmd->freeze_tlv_delay_cnt,
					rcc->freeze_tlv_delay_cnt_en);
	WMI_CFR_FREEZE_DELAY_CNT_THR_SET(cmd->freeze_tlv_delay_cnt,
					 rcc->freeze_tlv_delay_cnt_thr);
	WMI_CFR_DIRECTED_FTM_ACK_EN_SET(cmd->filter_type,
					rcc->m_directed_ftm);
	WMI_CFR_ALL_FTM_ACK_EN_SET(cmd->filter_type,
				   rcc->m_all_ftm_ack);
	WMI_CFR_NDPA_NDP_DIRECTED_EN_SET(cmd->filter_type,
					 rcc->m_ndpa_ndp_directed);
	WMI_CFR_NDPA_NDP_ALL_EN_SET(cmd->filter_type,
				    rcc->m_ndpa_ndp_all);
	WMI_CFR_TA_RA_TYPE_FILTER_EN_SET(cmd->filter_type,
					 rcc->m_ta_ra_filter);
	WMI_CFR_FILTER_IN_AS_FP_TA_RA_TYPE_SET(cmd->filter_type,
					       rcc->en_ta_ra_filter_in_as_fp);
	WMI_CFR_ALL_PACKET_EN_SET(cmd->filter_type,
				  rcc->m_all_packet);

	/* TLV indicating array of structures to follow */
	buf_ptr += sizeof(wmi_cfr_capture_filter_cmd_fixed_param);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       rcc->num_grp_tlvs * sizeof(wmi_cfr_filter_group_config));

	if (rcc->num_grp_tlvs) {
		buf_ptr += WMI_TLV_HDR_SIZE;
		param = (wmi_cfr_filter_group_config *)buf_ptr;

		for (grp_id = 0; grp_id < MAX_TA_RA_ENTRIES; grp_id++) {
			if (qdf_test_bit(grp_id,
					 &rcc->modified_in_curr_session)) {
				populate_wmi_cfr_param(grp_id, rcc, param);
				param++;
			}
		}
	}
	status = wmi_unified_cmd_send(wmi_handle, buf, len,
				      WMI_CFR_CAPTURE_FILTER_CMDID);
	if (status)
		wmi_buf_free(buf);

	return status;
}

static QDF_STATUS
extract_cfr_phase_param_tlv(wmi_unified_t wmi_handle,
			    void *evt_buf,
			    struct wmi_cfr_phase_delta_param *param)
{
	WMI_PDEV_AOA_PHASEDELTA_EVENTID_param_tlvs *param_buf;
	wmi_pdev_aoa_phasedelta_evt_fixed_param *phase_event;

	param_buf = (WMI_PDEV_AOA_PHASEDELTA_EVENTID_param_tlvs *)evt_buf;
	if (!param_buf) {
		wmi_err("Invalid cfr aoa phase delta buffer");
		return QDF_STATUS_E_INVAL;
	}

	phase_event = param_buf->fixed_param;
	if (!phase_event) {
		wmi_err("CFR phase AoA delta buffer is NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	param->freq = phase_event->freq;
	param->pdev_id = wmi_handle->ops->convert_pdev_id_target_to_host
				(wmi_handle, phase_event->pdev_id);

	param->max_chains = phase_event->chainInfo & 0xFFFF;

	param->chain_phase_mask = (phase_event->chainInfo >> 16) & 0xFFFF;

	if ((sizeof(param->ibf_cal_val)) <
	    (sizeof(phase_event->perChainIbfCalVal))) {
		wmi_err("ibf_cal_val can not hold all values from event data");
		return QDF_STATUS_E_RANGE;
	}

	if ((sizeof(param->phase_delta)) <
	    (sizeof(phase_event->phasedelta))) {
		wmi_err("phase_delta can not hold all values from event data");
		return QDF_STATUS_E_RANGE;
	}

	qdf_mem_copy(param->ibf_cal_val,
		     phase_event->perChainIbfCalVal,
		     sizeof(param->ibf_cal_val));

	qdf_mem_copy(param->phase_delta,
		     phase_event->phasedelta,
		     sizeof(param->phase_delta));

	return QDF_STATUS_SUCCESS;
}

#ifdef WLAN_RCC_ENHANCED_AOA_SUPPORT
static QDF_STATUS
extract_cfr_enh_phase_fixed_param_tlv
		(wmi_unified_t wmi_handle,
		 void *evt_buf,
		 struct wmi_cfr_enh_phase_delta_param *param)
{
	WMI_PDEV_ENHANCED_AOA_PHASEDELTA_EVENTID_param_tlvs *ev_buf;
	wmi_pdev_enhanced_aoa_phasedelta_evt_fixed_param *fixed_param;

	ev_buf = (WMI_PDEV_ENHANCED_AOA_PHASEDELTA_EVENTID_param_tlvs *)evt_buf;
	if (!ev_buf) {
		wmi_err("Invalid cfr enh aoa phase delta event buffer");
		return QDF_STATUS_E_INVAL;
	}

	fixed_param = ev_buf->fixed_param;
	if (!fixed_param) {
		wmi_err("cfr enh aoa event fixed_param is NULL");
		return QDF_STATUS_E_NULL_VALUE;
	}

	param->freq = fixed_param->freq;
	param->pdev_id = wmi_handle->ops->convert_pdev_id_target_to_host
				(wmi_handle, fixed_param->pdev_id);

	param->max_chains =
		WMI_AOA_MAX_SUPPORTED_CHAINS_GET(fixed_param->chain_info);
	param->data_for_chainmask =
		WMI_AOA_SUPPORTED_CHAINMASK_GET(fixed_param->chain_info);
	param->xbar_config = fixed_param->xbar_config;

	if (sizeof(param->ibf_cal_val) <
			sizeof(fixed_param->per_chain_ibf_cal_val)) {
		wmi_err("ibf_cal_val can not hold all values from event data");
		return QDF_STATUS_E_RANGE;
	}

	qdf_mem_copy(param->ibf_cal_val, fixed_param->per_chain_ibf_cal_val,
		     QDF_MIN(sizeof(param->ibf_cal_val),
			     sizeof(fixed_param->per_chain_ibf_cal_val)));

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS
populate_enhanced_aoa_data(uint32_t *dst_array, uint32_t *src_array,
			   wmi_enhanced_aoa_gain_phase_data_hdr *data_hdr,
			   uint32_t offset, uint32_t dst_size)
{
	uint32_t src_size = WMI_AOA_NUM_ENTIRES_GET(data_hdr->data_info) *
				sizeof(uint32_t);

	if (src_size > dst_size) {
		wmi_err("the amount of data can not fit in the host array");
		return QDF_STATUS_E_RANGE;
	}

	qdf_mem_copy(dst_array, src_array + offset, src_size);

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS
extract_cfr_enh_phase_data_tlv(wmi_unified_t wmi_handle,
			       void *evt_buf,
			       struct wmi_cfr_enh_phase_delta_param *param)
{
	WMI_PDEV_ENHANCED_AOA_PHASEDELTA_EVENTID_param_tlvs *ev_buf;
	wmi_enhanced_aoa_gain_phase_data_hdr *data_hdr;
	QDF_STATUS status;
	uint32_t *dst_array = NULL;
	uint32_t i, offset = 0;
	uint8_t data_type;

	ev_buf = (WMI_PDEV_ENHANCED_AOA_PHASEDELTA_EVENTID_param_tlvs *)evt_buf;

	if (!ev_buf) {
		wmi_err("Invalid cfr enh aoa phase delta event buffer");
		return QDF_STATUS_E_INVAL;
	}

	if (!ev_buf->aoa_data_hdr) {
		wmi_err("data headers NULL.. investigate");
		return QDF_STATUS_E_NULL_VALUE;
	}

	if (!ev_buf->aoa_data_buf) {
		wmi_err("data bufs NULL.. investigate");
		return QDF_STATUS_E_NULL_VALUE;
	}

	for (i = 0; i < ev_buf->num_aoa_data_hdr; i++) {
		data_hdr = &ev_buf->aoa_data_hdr[i];
		data_type = WMI_AOA_DATA_TYPE_GET(data_hdr->data_info);

		if (data_type == WMI_PHASE_DELTA_ARRAY) {
			dst_array = param->enh_phase_delta_array;
		} else if (data_type == WMI_GAIN_GROUP_STOP_ARRAY) {
			dst_array = param->gain_stop_index_array;
		} else {
			wmi_err("invalid aoa data type received");
			return QDF_STATUS_E_INVAL;
		}

		status = populate_enhanced_aoa_data
				(dst_array, ev_buf->aoa_data_buf,
				 data_hdr, offset, param->array_size);
		if (status) {
			wmi_err("error in populating aoa data");
			return status;
		}

		offset += WMI_AOA_NUM_ENTIRES_GET(data_hdr->data_info);
	}

	return QDF_STATUS_SUCCESS;
}
#endif /* WLAN_RCC_ENHANCED_AOA_SUPPORT */
#endif /* WLAN_ENH_CFR_ENABLE */

static QDF_STATUS send_peer_cfr_capture_cmd_tlv(wmi_unified_t wmi_handle,
						struct peer_cfr_params *param)
{
	wmi_peer_cfr_capture_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int len = sizeof(*cmd);
	int ret;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		qdf_print("%s:wmi_buf_alloc failed\n", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_peer_cfr_capture_cmd_fixed_param *)wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_cfr_capture_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_peer_cfr_capture_cmd_fixed_param));

	WMI_CHAR_ARRAY_TO_MAC_ADDR(param->macaddr, &cmd->mac_addr);
	cmd->request = param->request;
	cmd->vdev_id = param->vdev_id;
	cmd->periodicity = param->periodicity;
	cmd->bandwidth = param->bandwidth;
	cmd->capture_method = param->capture_method;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_PEER_CFR_CAPTURE_CMDID);
	if (QDF_IS_STATUS_ERROR(ret)) {
		wmi_err("Failed to send WMI_PEER_CFR_CAPTURE_CMDID");
		wmi_buf_free(buf);
	}

	return ret;
}

#ifdef WLAN_ENH_CFR_ENABLE
static inline void wmi_enh_cfr_attach_tlv(wmi_unified_t wmi_handle)
{
	struct wmi_ops *ops = wmi_handle->ops;

	ops->send_cfr_rcc_cmd = send_cfr_rcc_cmd_tlv;
}
#else
static inline void wmi_enh_cfr_attach_tlv(wmi_unified_t wmi_handle)
{
}
#endif

#ifdef WLAN_RCC_ENHANCED_AOA_SUPPORT
static void wmi_cfr_attach_enh_aoa_tlv(struct wmi_ops *ops)
{
	ops->extract_cfr_enh_phase_data = extract_cfr_enh_phase_data_tlv;
	ops->extract_cfr_enh_phase_fixed_param =
		extract_cfr_enh_phase_fixed_param_tlv;
}
#else
static void wmi_cfr_attach_enh_aoa_tlv(struct wmi_ops *ops)
{
}
#endif /* WLAN_RCC_ENHANCED_AOA_SUPPORT */

void wmi_cfr_attach_tlv(wmi_unified_t wmi_handle)
{
	struct wmi_ops *ops = wmi_handle->ops;

	ops->send_peer_cfr_capture_cmd = send_peer_cfr_capture_cmd_tlv;
	ops->extract_cfr_peer_tx_event_param =
		extract_cfr_peer_tx_event_param_tlv;
	ops->extract_cfr_phase_param = extract_cfr_phase_param_tlv;
	wmi_cfr_attach_enh_aoa_tlv(ops);
	wmi_enh_cfr_attach_tlv(wmi_handle);
}
#endif /* WLAN_CFR_ENABLE */

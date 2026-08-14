// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0


// u_system_devices_static_finalize
/*!
 * @file
 * @brief  Found function for diy vr headset based off hdk_prober.c
 * @author Adapted by Simon Read
 * @ingroup drv_diy_vr
 */

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_logging.h"

#include "diy_interface.h"

int
diy_vr_found(struct xrt_prober *xp,
		  struct xrt_prober_device **devices,
		  size_t device_count,
		  size_t index,
		  cJSON *attached_data,
		  struct xrt_device **out_xdev) {

	U_LOG_T("%s - !!!!!!! Entered diy vr found function", __func__);
	printf("Entered diy_vr_found");

	struct xrt_prober_device *dev = devices[index];

	struct os_hid_device *hid = NULL;

	int result = xrt_prober_open_hid_interface(xp, dev, ARDUINO_IFACE, &hid);
	if (result != 0) {
		U_LOG_E("%s - Failed to open Arduino HID interface", __func__);
		return -1;
	}
	struct diy_vr *hmd = diy_vr_create(hid);
	if (hmd == NULL) {
		U_LOG_E("%s - diy_vr_create failed", __func__);
		return -1;
	}
	*out_xdev = &hmd->base;
	return 1;
}

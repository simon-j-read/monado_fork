// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  "auto-prober" for Sample HMD that can be autodetected but not through USB VID/PID.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_diy_vr
 */

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"

#include "diy_interface.h"


int
diy_vr_found(struct xrt_prober *xp,
		  struct xrt_prober_device **devices,
		  size_t device_count,
		  size_t index,
		  cJSON *attached_data,
		  struct xrt_device **out_xdev) {

	struct xrt_prober_device *dev = devices[index];

	unsigned char buf[256] = {0};
	int result = xrt_prober_get_string_descriptor(xp, dev, XRT_PROBER_STRING_PRODUCT, buf, sizeof(buf));

	enum HDK_VARIANT variant = HDK_UNKNOWN;
	const char *name = NULL;
	if (0 == strncmp(HDK2_PRODUCT_STRING, (const char *)buf, sizeof(buf))) {
		variant = HDK_VARIANT_2;
		name = HDK2_PRODUCT_STRING;
	} else if (0 == strncmp(HDK1_PRODUCT_STRING, (const char *)buf, sizeof(buf))) {
		variant = HDK_VARIANT_1_2;
		name = HDK12_PRODUCT_STRING;
	} else {
		//! @todo just assuming anything else is 1.3 for now
		variant = HDK_VARIANT_1_3_1_4;
		name = HDK13_PRODUCT_STRING;
	}

	U_LOG_I("%s - Found at least the tracker of some HDK (%s) -- opening\n", __func__, name);

	struct os_hid_device *hid = NULL;
	// Interface 2 is the HID interface.
	result = xrt_prober_open_hid_interface(xp, dev, 2, &hid);
	if (result != 0) {
		return -1;
	}
	struct hdk_device *hd = hdk_device_create(hid, variant);
	if (hd == NULL) {
		return -1;
	}
	*out_xdev = &hd->base;
	return 1;
}


/*!
 * @implements xrt_auto_prober
 */
struct diy_vr_auto_prober
{
	struct xrt_auto_prober base;
};

//! @private @memberof diy_vr_auto_prober
static inline struct diy_vr_auto_prober *
diy_vr_auto_prober(struct xrt_auto_prober *xap)
{
	return (struct diy_vr_auto_prober *)xap;
}

//! @private @memberof diy_vr_auto_prober
static void
diy_vr_auto_prober_destroy(struct xrt_auto_prober *p)
{
	struct diy_vr_auto_prober *ap = diy_vr_auto_prober(p);

	free(ap);
}

//! @public @memberof diy_vr_auto_prober
static int
diy_vr_auto_prober_autoprobe(struct xrt_auto_prober *xap,
                             cJSON *attached_data,
                             bool no_hmds,
                             struct xrt_prober *xp,
                             struct xrt_device **out_xdevs)
{
	struct diy_vr_auto_prober *ap = diy_vr_auto_prober(xap);
	(void)ap;

	// Do not create an HMD device if we are not looking for HMDs.
	if (no_hmds) {
		return 0;
	}

	out_xdevs[0] = diy_vr_create();
	return 1;
}

struct xrt_auto_prober *
diy_vr_create_auto_prober(void)
{
	struct diy_vr_auto_prober *ap = U_TYPED_CALLOC(struct diy_vr_auto_prober);
	ap->base.name = "DIY VR HMD Auto-Prober";
	ap->base.destroy = diy_vr_auto_prober_destroy;
	ap->base.lelo_dallas_autoprobe = diy_vr_auto_prober_autoprobe;

	return &ap->base;
}

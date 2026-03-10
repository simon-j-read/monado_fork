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

#include "diy_vr_interface.h"


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
	ap->base.name = "Sample HMD Auto-Prober";
	ap->base.destroy = diy_vr_auto_prober_destroy;
	ap->base.lelo_dallas_autoprobe = diy_vr_auto_prober_autoprobe;

	return &ap->base;
}

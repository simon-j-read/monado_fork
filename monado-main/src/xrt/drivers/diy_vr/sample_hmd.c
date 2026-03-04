// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Sample HMD device, use as a starting point to make your own device driver.
 *
 *
 * Based largely on simulated_hmd.c
 *
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_diy_vr
 */

#include "os/os_time.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_relation_history.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"
#include "xrt/xrt_results.h"

#include <stdio.h>

#define CONFIG_SUCCESS 1;
#define CONFIG_FAILURE 0;


/*
 *
 * Structs and defines.
 *
 */

/*!
 * A diy_vr HMD device.
 *
 * @implements xrt_device
 */
struct diy_vr
{
	struct xrt_device base;

	struct xrt_pose pose;

	enum u_logging_level log_level;

	// has built-in mutex so thread safe
	struct m_relation_history *relation_hist;
};


/// Casting helper function
static inline struct diy_vr *
diy_vr(struct xrt_device *xdev)
{
	return (struct diy_vr *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(diy_vr_log, "DIY_VR_LOG", U_LOGGING_WARN)

#define HMD_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

static void
diy_vr_destroy(struct xrt_device *xdev)
{
	struct diy_vr *hmd = diy_vr(xdev);

	// Remove the variable tracking.
	u_var_remove_root(hmd);


	m_relation_history_destroy(&hmd->relation_hist);

	u_device_free(&hmd->base);
}

static xrt_result_t
diy_vr_update_inputs(struct xrt_device *xdev)
{
	/*
	 * Empty for the diy_vr driver, if you need to you should
	 * put code to update the attached inputs fields. If not you can use
	 * the u_device_noop_update_inputs helper to make it a no-op.
	 */
	return XRT_SUCCESS;
}

static xrt_result_t
diy_vr_get_tracked_pose(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            int64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	struct diy_vr *hmd = diy_vr(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;

	enum m_relation_history_result history_result =
	    m_relation_history_get(hmd->relation_hist, at_timestamp_ns, &relation);
	if (history_result == M_RELATION_HISTORY_RESULT_INVALID) {
		// If you get in here, it means you did not push any poses into the relation history.
		// You may want to handle this differently.
		HMD_ERROR(hmd, "Internal error: no poses pushed?");
	}

	if ((relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		// If we provide an orientation, make sure that it is normalized.
		math_quat_normalize(&relation.pose.orientation);
	}

	*out_relation = relation;
	return XRT_SUCCESS;
}

static xrt_result_t
diy_vr_get_view_poses(struct xrt_device *xdev,
                          const struct xrt_vec3 *default_eye_relation,
                          int64_t at_timestamp_ns,
                          enum xrt_view_type view_type,
                          uint32_t view_count,
                          struct xrt_space_relation *out_head_relation,
                          struct xrt_fov *out_fovs,
                          struct xrt_pose *out_poses)
{
	/*
	 * For HMDs you can call this function or directly set
	 * the `get_view_poses` function on the device to it.
	 */
	return u_device_get_view_poses( //
	    xdev,                       //
	    default_eye_relation,       //
	    at_timestamp_ns,            //
	    view_type,                  //
	    view_count,                 //
	    out_head_relation,          //
	    out_fovs,                   //
	    out_poses);                 //
}

static xrt_result_t
diy_vr_get_visibility_mask(struct xrt_device *xdev,
                               enum xrt_visibility_mask_type type,
                               uint32_t view_index,
                               struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}


void config_hmd_base(struct diy_vr *hmd)
/*
 * Performs initial config of the base aspect of the hmd
 */
{
	// This list should be ordered, most preferred first.
	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	hmd->base.update_inputs = diy_vr_update_inputs;
	hmd->base.get_tracked_pose = diy_vr_get_tracked_pose;
	hmd->base.get_view_poses = diy_vr_get_view_poses;
	hmd->base.get_visibility_mask = diy_vr_get_visibility_mask;
	hmd->base.destroy = diy_vr_destroy;
}

void config_hmd_inputs(struct diy_vr *hmd)
/*
 *
 */
{
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = true;
}

int check_fovs(struct diy_vr *hmd, const double hCOP, const double vCOP, const double hFOV, const double vFOV)
/*
 *  Check the FOVs to make sure that the math works to allow for half-FOVs
 */
{
	int compute_outcome;
	if (
	/* right eye */
	!math_compute_fovs(1, hCOP, hFOV, 1, vCOP, vFOV, &hmd->base.hmd->distortion.fov[1]) ||
	/*
	 * left eye - same as right eye, except the horizontal center of projection is moved in the opposite
	 * direction now
	 */
	!math_compute_fovs(1, 1.0 - hCOP, hFOV, 1, vCOP, vFOV, &hmd->base.hmd->distortion.fov[0]))
	{
		// If those failed, it means our math was impossible.
		compute_outcome = CONFIG_FAILURE;
	}
	else {
		compute_outcome = CONFIG_SUCCESS;
	}

	return compute_outcome;
}

/*
 * Configurate the display aspect of the HMD.
 * Refresh rate, FOV, COP, Panel width and height.
 * Checking that the half-FOVs work
 */
int
config_hmd_display(struct diy_vr *hmd)
{
	// TODO
	// iterate through display config settings
	// assign values based off what is in JSON config file

	// Things to put in JSON config
	const float display_refresh_hz = 60.0;	// Refresh rate of the display in the HMD
	const double hFOV_deg = 90;				// Field of views
	const double vFOV_deg = 96.73;			//
	const double hCOP = 0.529;				// Center of projections
	const double vCOP = 0.5;				//
	const int panel_w = 1080;
	const int panel_h = 1200;

	// Assign refresh rate (nanoseconds) (1/Hz = seconds)
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / display_refresh_hz);

	const double hFOV_rad = DEG_TO_RAD(hFOV_deg); // math_computer_fovs needs it in radians
	const double vFOV_rad = DEG_TO_RAD(vFOV_deg); // from m_api.h

	 if (!check_fovs(hmd, hCOP, hFOV_rad, vCOP, vFOV_rad)) {
		 HMD_ERROR(hmd, "Failed to setup basic device info: fov calculations");
	 	return CONFIG_FAILURE; // fov math did not compute correctly
	 }

	hmd->base.hmd->screens[0].w_pixels = panel_w * 2;	// Have 2 panels, but treated as a single screen by Monado
	hmd->base.hmd->screens[0].h_pixels = panel_h;		// Single "screen" (always the case), hence [0] index.

	// Left, Right
	for (uint8_t eye = 0; eye < 2; ++eye) {
		hmd->base.hmd->views[eye].display.w_pixels = panel_w;	// Display
		hmd->base.hmd->views[eye].display.h_pixels = panel_h;

		hmd->base.hmd->views[eye].viewport.y_pixels = 0;		// Viewport
		hmd->base.hmd->views[eye].viewport.w_pixels = panel_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = panel_h;
		// if rotation is not identity, the dimensions can get more complex.
		hmd->base.hmd->views[eye].rot = u_device_rotation_ident;
	}
	// left eye starts at x=0, right eye starts at x=panel_width
	hmd->base.hmd->views[0].viewport.x_pixels = 0;
	hmd->base.hmd->views[1].viewport.x_pixels = panel_w;

	return CONFIG_SUCCESS;
}

struct xrt_device *
diy_vr_create(void)
{
	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct diy_vr *hmd = U_DEVICE_ALLOCATE(struct diy_vr, flags, 1, 0);

	// Perform initial population of hmd->base.hmd... values
	// Blending, inputs, poses, visibility mask & destroy function.
	config_hmd_base(hmd);

	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&hmd->base);

	// populate this with something more complex if required
	// hmd->base.compute_distortion = diy_vr_compute_distortion;

	hmd->pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	hmd->log_level = debug_get_log_option_diy_vr_log();

	// Print name.
	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Sample HMD");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "Sample HMD S/N");

	m_relation_history_create(&hmd->relation_hist);

	config_hmd_inputs(hmd);  // TODO COMMENT

	if (config_hmd_display(hmd) == CONFIG_FAILURE) // Config display, FOV calculations can fail. Ensure correct config
	{
		diy_vr_destroy(&hmd->base);
		return NULL;
	}



	// Distortion information, fills in xdev->compute_distortion().
	u_distortion_mesh_set_none(&hmd->base);

	// Just put an initial identity value in the tracker
	struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	identity.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                          XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
	uint64_t now = os_monotonic_get_ns();
	m_relation_history_push(hmd->relation_hist, &identity, now);

	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(hmd, "Sample HMD", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");


	return &hmd->base;
}

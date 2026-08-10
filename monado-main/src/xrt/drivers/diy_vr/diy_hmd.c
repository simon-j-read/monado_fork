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

#include "math/m_relation_history.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep

#include "os/os_hid.h"
#include "os/os_time.h"
#include "os/os_threading.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_results.h"

#include <stdio.h>
#include <string.h>

#include "read_config.h"
#include "diy_interface.h"

#define SUCCESS 1
#define FAILURE 0

/*!
 * A parsed sample of accel and gyro.
 */
struct arduino_parsed_sample
{
	uint32_t time;
	uint32_t delta;
	struct xrt_vec3 accel;
	struct xrt_vec3 gyro;
};

struct arduino_parsed_input
{
	uint32_t timestamp;
	struct arduino_parsed_sample sample;
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
#define HMD_WARN(hmd, ...) U_LOG_XDEV_IFL_W(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)


static uint32_t
calc_delta_and_handle_rollover(uint32_t next, uint32_t last)
{
	uint32_t tick_delta = next - last;

	// The 24-bit tick counter has rolled over,
	// adjust the "negative" value to be positive.
	if (tick_delta > 0xffffff) {
		tick_delta += 0x1000000;
	}

	return tick_delta;
}

static void
diy_vr_destroy(struct xrt_device *xdev)
{
	struct diy_vr *hmd = diy_vr(xdev);

	// Remove the variable tracking.
	u_var_remove_root(hmd);

	// Destroy the thread object.
	os_thread_helper_destroy(&hmd->imu_thread);

	// Now that the thread is not running we can destroy the lock.
	os_mutex_destroy(&hmd->lock);

	// Destroy the fusion.
	m_imu_3dof_close(&hmd->fusion);

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

/*
 * A consumer of pose data
 * See "arduino_run_thread" for the "updater" of the pose data.
 *
 * This will be used by other parts of monado to get the HMD's current pose
 * I believe, hence why there needs to be thread locking in certain parts to
 * prevent the thread that is updating the pose to conflict with the thread
 * that is reading the pose.
 */
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


	os_mutex_lock(&hmd->lock);

	out_relation->pose.orientation = hmd->fusion.rot;
	math_quat_normalize(&out_relation->pose.orientation); // should this happen?

	os_mutex_unlock(&hmd->lock);

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

// Configure the blend modes for the HMD, currently just have one.
// But pretty sure you can have more, based off the comment provided by Monado's authors.
void
config_hmd_blend_modes(struct diy_vr *hmd) {
	/*
	The possible blend modes are specified in xrt_defines.h:
		XRT_BLEND_MODE_OPAQUE		- Typical for VR experiences.
		XRT_BLEND_MODE_ADDITIVE		- Typical for an AR experience on a see-through headset with an additive display,
		XRT_BLEND_MODE_ALPHA_BLEND	- Typical for an AR experience on a phone or headset that supports video passthrough.
	*/
	// This list should be ordered, most preferred first.
	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;  // TODO What is this?
	hmd->base.hmd->blend_mode_count = idx;						// Only applied one blend mode
}

// Configure the "methods" of the inherited hmd->base.
// This is done by loading the functions defined above into the various methods.
void
config_hmd_functions(struct diy_vr *hmd)
{
	// Assigning the HMD specific functions
	hmd->base.update_inputs = diy_vr_update_inputs;
	hmd->base.get_tracked_pose = diy_vr_get_tracked_pose;
	hmd->base.get_view_poses = diy_vr_get_view_poses;
	hmd->base.get_visibility_mask = diy_vr_get_visibility_mask;
	hmd->base.destroy = diy_vr_destroy;
}

/*
 * Init of inputs for HMD
 * TODO - See how this integrates with the other helper functions of main. i.e., how much does the order matter.
 */
void
config_hmd_inputs(struct diy_vr *hmd)
{
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = true;
}

/*
 *  Check the FOVs to make sure that the math works to allow for half-FOVs
 */
int
check_fovs(struct diy_vr *hmd, const double hCOP, const double vCOP, const double hFOV, const double vFOV)

{
	int compute_outcome = SUCCESS;
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
		compute_outcome = FAILURE;
	}

	return compute_outcome;
}

/*
 * Configurate the display aspect of the HMD.
 * Refresh rate, FOV, COP, Panel width and height.
 * Checking that the half-FOVs work
 */
int
config_hmd_display(struct diy_vr *hmd, struct diy_config_data *cd)
{
	// Assign refresh rate (nanoseconds) (1/Hz = seconds)
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / cd->display_refresh_hz);

	const double hFOV_rad = DEG_TO_RAD(cd->hFOV_deg); // math_computer_fovs needs it in radians
	const double vFOV_rad = DEG_TO_RAD(cd->vFOV_deg); // from m_api.h

	 if (!check_fovs(hmd, cd->hCOP, cd->vCOP, hFOV_rad, vFOV_rad)) {
		 HMD_ERROR(hmd, "Failed to setup basic device info: fov calculations");
	 	return FAILURE; // fov math did not compute correctly
	 }

	hmd->base.hmd->screens[0].w_pixels = cd->panel_w * 2;	// Have 2 panels, but treated as a single screen by Monado
	hmd->base.hmd->screens[0].h_pixels = cd->panel_h;		// Single "screen" (always the case), hence [0] index.

	// Left, Right
	for (uint8_t eye = 0; eye < 2; ++eye) {
		hmd->base.hmd->views[eye].display.w_pixels = cd->panel_w;	// Display
		hmd->base.hmd->views[eye].display.h_pixels = cd->panel_h;

		hmd->base.hmd->views[eye].viewport.y_pixels = 0;		// Viewport
		hmd->base.hmd->views[eye].viewport.w_pixels = cd->panel_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = cd->panel_h;
		// if rotation is not identity, the dimensions can get more complex.
		hmd->base.hmd->views[eye].rot = u_device_rotation_ident;
	}
	// left eye starts at x=0, right eye starts at x=panel_width
	hmd->base.hmd->views[0].viewport.x_pixels = 0;
	hmd->base.hmd->views[1].viewport.x_pixels = cd->panel_w;

	return SUCCESS;
}


/*
 * m_relation_history contains a buffer of previous poses for the HMD.
 * Here we are giving the buffer an update, by pushing an 'identity' pose onto the buffer.
 * I imagine this gives the HMD a clean slate at start up, may need to play around with this.
 */
void
config_hmd_relation_hist(struct diy_vr *hmd)
{
	// hmd->relation_hist already initialised in main, unsure if i can put it in this function or if it needs to be
	// called eariler in main.
	// Just put an initial identity value in the tracker (tracker being hmd->relation_hist)
	struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	identity.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
															  XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);

	uint64_t now = os_monotonic_get_ns();							// Pushing to buffer requires a timestamp
	m_relation_history_push(hmd->relation_hist, &identity, now);	// Pushing identity onto the HMD's pose history buffer
}


/*
 *
 */
static void
update_fusion(struct diy_vr *hmd,
			  struct arduino_parsed_sample *sample,
			  timepoint_ns timestamp_ns,
			  time_duration_ns delta_ns)
{

	//hmd->device_time += (uint64_t)sample->delta * 1000;

	m_imu_3dof_update(&hmd->fusion, timestamp_ns, &sample->accel, &sample->gyro);

	double delta_device_ms = (double)sample->delta / 1000.0;
	double delta_host_ms = (double)delta_ns / (1000.0 * 1000.0);
	HMD_DEBUG(hmd, "%+fms %+fms", delta_host_ms, delta_device_ms);
	HMD_DEBUG(hmd, "fusion sample %u (ax %.3f ay %.3f az %.3f) (gx %.3f gy %.3f gz %.3f)",
			  sample->time,
			  sample->accel.x, sample->accel.y, sample->accel.z,
			  sample->gyro.x, sample->gyro.y, sample->gyro.z);
	HMD_DEBUG(hmd, " ");
}


/*
 * From data
 */
static void
arduino_parse_input(struct diy_vr *hmd, void *data, struct arduino_parsed_input *input)
{
	U_ZERO(input);
	const int bytes_4 = 4;
	unsigned char *b = (unsigned char *)data;
	// Raw HID visualisation
	HMD_TRACE(hmd,
				  "raw input: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
				  "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
				  "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				  b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14],
				  b[15], b[16], b[17], b[18], b[19], b[20], b[21], b[22], b[23], b[24], b[25], b[26], b[27], b[28],
				  b[29], b[30], b[31], b[32], b[33], b[34], b[35]);

	// Accelerations
	memcpy(&input->sample.accel.x, &b[0], bytes_4);
	memcpy(&input->sample.accel.y, &b[4], bytes_4);
	memcpy(&input->sample.accel.z, &b[8], bytes_4);

	// Gyro
	memcpy(&input->sample.gyro.x, &b[12], bytes_4);
	memcpy(&input->sample.gyro.y, &b[16], bytes_4);
	memcpy(&input->sample.gyro.z, &b[20], bytes_4);

	// Arduino time since program start (difference from last sample)
	memcpy(&input->sample.time, &b[24], bytes_4);
	input->sample.delta = calc_delta_and_handle_rollover(time, hmd->last_time);
	hmd->last_time = input->sample.time;

	// Parsed HID visualisation
	HMD_TRACE(hmd, "parsed: accel(%.3f, %.3f, %.3f) gyro(%.3f, %.3f, %.3f) t=%u",
		  input->sample.accel.x, input->sample.accel.y, input->sample.accel.z,
		  input->sample.gyro.x, input->sample.gyro.y, input->sample.gyro.z,
		  input->sample.time);
}


/*!
 * Reads one packet from the device,handles locking and checking if
 * the thread has been told to shut down.
 * 
 * Read data goes into "buffer" which will be parsed outside this function.
 */
static bool
arduino_read_one_packet(struct diy_vr *hmd, uint8_t *buffer)
{
	// Wait for something to come through HID (might need to play with block timer)
	const uint8_t block_ms = 100;
	int bytesRead = os_hid_read(hmd->dev, buffer, PACKET_SIZE, block_ms);

	// Something wrong, shouldn't get negative in normal operating mode.
	// I think it indicates a disconnection
	if (bytesRead < 0) {
		if (!hmd->disconnect_notified) {
			HMD_ERROR(hmd, "Negative bytes read from HID, not good. Arduino disconnected?");
			hmd->disconnect_notified = true;
		}
		return FAILURE;

	// No data in HID buffer even after waiting, won't update pose this time.
	// Might need to adjust Arduino output if this happens too often.
	} else if (bytesRead == 0) {
		HMD_WARN(hmd, "Read 0 bytes from device");
		return SUCCESS;
	}
	// Now that something is in the buffer, loop through till you get the lastest packet.
	// Will exit loop with latest packets saved into bytesread.
	while (bytesRead > 0) {
		if (bytesRead != PACKET_SIZE) {
			HMD_DEBUG(hmd, "Only got %d bytes", bytesRead);
			return SUCCESS;
		}
		bytesRead = os_hid_read(hmd->dev, buffer, PACKET_SIZE, 0); // Block 0, means 0 polls.
	}

	return SUCCESS;
}


/*
 * This helper thread is responsible for reading the HID data from the arduino
 * one packet at a time. Parsing that data into the required format to be fed
 * into Monado's fusion functions for IMUs. These will update the pose of the HMD.
 * After which can be used by getter functions like get_tracked_pose
 */
static void *
arduino_run_thread(void *ptr)
{
	struct diy_vr *hmd = diy_vr((struct xrt_device *)ptr);
	uint8_t buffer[PACKET_SIZE];
	timepoint_ns then_ns;
	timepoint_ns now_ns;
	struct arduino_parsed_input input; // = {0};

	// wait for a package to sync up, it's discarded but that's okay.
	if (!arduino_read_one_packet(hmd, buffer)) {
		return NULL;
	}

	then_ns = os_monotonic_get_ns();
	while (arduino_read_one_packet(hmd, buffer)) {

		// As close to when we get a packet.
		now_ns = os_monotonic_get_ns();

		// Parse the data we got.
		arduino_parse_input(hmd, buffer, &input);

		time_duration_ns delta_ns = now_ns - then_ns;
		then_ns = now_ns;

		// Lock last and the fusion.
		os_mutex_lock(&hmd->lock);

		// Process the parsed data.
		update_fusion(hmd, &input.sample, now_ns, delta_ns);

		// Now done.
		os_mutex_unlock(&hmd->lock);
	}
	return NULL;
}


/*
*	TODO - Complete refactor of the function
* 		(Breaking it up into bite size pieces and figuring out what it all means and does)
*/
struct diy_vr *
diy_vr_create(struct os_hid_device *dev)
{
	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	struct diy_vr *hmd = U_DEVICE_ALLOCATE(struct diy_vr, flags, 1, 0);
	hmd->log_level = debug_get_log_option_diy_vr_log();

	if (config_hid(hmd) == FAILURE)	{
		diy_vr_destroy(&hmd->base);
		return NULL;
	}
	config_hmd_blend_modes(hmd);			// Placing Opaque blend mode in.
	config_hmd_functions(hmd); 				// Assigning custom functions for inherited HMD->base
	u_distortion_mesh_set_none(&hmd->base); // Distortion information, fills in xdev->compute_distortion().
	// populate this with something more complex if required
	// hmd->base.compute_distortion = diy_vr_compute_distortion;
	hmd->pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	
	// Load config data (saves having to rebuild the project everytime we alter the HMD).
	// Just putting some dummy values in below before actually loading the JSON in.
	struct diy_config_data config_data = {
		.name = "Undefined", .serial = "Undefined", .display_refresh_hz = 60.0,
		.hFOV_deg = 100.0, .vFOV_deg = 100.0, .hCOP = 0.5, .vCOP = 0.5,
		.panel_w = 1080, .panel_h = 1080
	};

	const char *file_path = "/home/simon/Documents/XR/monado_fork/monado-main/src/xrt/drivers/diy_vr/config.json"; // TODO make this adaptable.
	const cJSON *config_json = read_config_file(file_path); // Interpret file into a JSON format
	extract_config_data(&config_data, config_json);

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "%s", config_data.name);	// Assigning names to base.str & base.serial
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%s", config_data.serial);

	m_relation_history_create(&hmd->relation_hist);		// Enables history of poses of hmd, where has the hmd been in space.
	config_hmd_inputs(hmd);  							// TODO COMMENT

	// Run config display, FOV calculations can fail. Ensure correct config
	if (config_hmd_display(hmd, &config_data) == FAILURE)	{
		diy_vr_destroy(&hmd->base);
		return NULL;
	}

	u_distortion_mesh_set_none(&hmd->base); // Distortion information, fills in xdev->compute_distortion().
	config_hmd_relation_hist(hmd);			// Config the already initialised HMD pose history buffer.

	// =================================================================================================================
	// IMU Tracking
	m_imu_3dof_init(&hmd->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_300MS); //Options are 300MS/20MS don't really know the difference.

	int ret = os_thread_helper_init(&hmd->imu_thread);
	if (ret != 0) {
		HMD_ERROR(hmd, "Failed to start imu thread!");
		diy_vr_destroy(&hmd->base);
		return 0;
	}
	if (hmd->dev) {
		// Mutex before thread.
		ret = os_mutex_init(&hmd->lock);
		if (ret != 0) {
			HMD_ERROR(hmd, "Failed to init mutex!");
			diy_vr_destroy(&hmd->base);
			return NULL;
		}
	ret = os_thread_helper_start(&hmd->imu_thread, arduino_run_thread, hmd);
	if (ret != 0) {
		HMD_ERROR(hmd, "Failed to start thread!");
		diy_vr_destroy(&hmd->base);
		return NULL;
	}
	// =================================================================================================================
	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(hmd, "Sample HMD", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");

	return &hmd->base;
}

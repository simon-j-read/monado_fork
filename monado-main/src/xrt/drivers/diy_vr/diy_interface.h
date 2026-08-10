// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Sample HMD driver.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_diy_vr
 */

#pragma once

#include "os/os_threading.h"
#include "math/m_imu_3dof.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_diy_vr Sample HMD driver
 * @ingroup drv
 *
 * @brief Driver for a Sample HMD.
 *
 * Does no actual work.
 * Assumed to not be detectable by USB VID/PID,
 * and thus exposes an "auto-prober" to explicitly discover the device.
 *
 * See @ref writing-driver for additional information.
 *
 * This device has an implementation of @ref xrt_auto_prober to perform hardware
 * detection, as well as an implementation of @ref xrt_device for the actual device.
 *
 * If your device is or has USB HID that **can** be detected based on USB VID/PID,
 * you can skip the @ref xrt_auto_prober implementation, and instead implement a
 * "found" function that matches the signature expected by xrt_prober_entry::found.
 * See for example @ref hdk_found.
 * Alternately, you might create a builder or an instance implementation directly.
 */

// VID & PID Values for the Arduino HID, will need to be changed for your respective
// arduino.
#define DIY_VR_VID 0x2431
#define DIY_VR_PID 0x8036

#define PACKET_SIZE 36 // Number of bytes in Arduino HID packet.

/*!
 * A diy_vr HMD device.
 *
 * @implements xrt_device
 */
struct diy_vr
{
    struct xrt_device base;		// Core of the struct

    struct xrt_pose pose;					  // What handles all the poses
    struct m_relation_history *relation_hist; // (pose buffer) has built-in mutex so thread safe

    struct os_hid_device *dev;		    // Arduino Pro Micro HID with Adafruit IMU20948
    struct os_thread_helper imu_thread; // Used to read IMU sensor packets via Arduino HID.
    struct os_mutex lock;				// Thread locking stuff
    bool disconnect_notified;           // For notification that we aren't reading anything from HID

    uint32_t last_time;
    struct m_imu_3dof fusion;

    enum u_logging_level log_level;

};


/*
 * Probing function for diy vr devices following the diy vr setup.
 *
 * @ingroup drv_diy_vr
 * @see xrt_prober_found_func_t
 */
int
diy_vr_found(struct xrt_prober *xp,
          struct xrt_prober_device **devices,
          size_t device_count,
          size_t index,
          cJSON *attached_data,
          struct xrt_device **out_xdev);


/*!
 * Create a Sample HMD.
 *
 * This is only exposed so that the prober (in one source file)
 * can call the construction function (in another)
 * @ingroup drv_diy_vr
 */
struct diy_vr *
diy_vr_create(struct os_hid_device *dev);

/*!
 * @dir drivers/diy_vr
 *
 * @brief @ref drv_diy_vr files.
 */


#ifdef __cplusplus
}
#endif

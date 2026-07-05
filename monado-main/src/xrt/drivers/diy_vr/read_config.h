//
// Created by simon on 09/03/2026.
//

#ifndef XRT_READ_CONFIG_H
#define XRT_READ_CONFIG_H

#include <cjson/cJSON.h>

struct diy_config_data
{
    char *name;               // Name of HMD (may need to be EDID for compositor leasing)
    char *serial;             // Serial of HMD (may need to be EDID for compositor leasing)
    float display_refresh_hz;	// Refresh rate of the display in the HMD
    double hFOV_deg;			// Field of views
    double vFOV_deg;			// ""
    double hCOP;				// Center of projections
    double vCOP;				// ""
    int panel_w;              // In pixels
    int panel_h;              // ""
};

const cJSON * read_config_file(const char *file_path);


void
extract_config_data(struct diy_config_data *config_data, const cJSON *root);

#endif //XRT_READ_CONFIG_H
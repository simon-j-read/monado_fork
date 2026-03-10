// Module is holds helper functions to read the config JSON for the DIY HMD
// Created by simon on 09/03/2026.
//

#include "read_config.h"

#include "util/u_file.h"
#include "util/u_json.h"
#include <cjson/cJSON.h>

struct diy_config_data
{
    const char *name;               // Name of HMD (may need to be EDID for compositor leasing)
    const char *serial;             // Serial of HMD (may need to be EDID for compositor leasing)
    const float display_refresh_hz;	// Refresh rate of the display in the HMD
    const double hFOV_deg;			// Field of views
    const double vFOV_deg;			// ""
    const double hCOP;				// Center of projections
    const double vCOP;				// ""
    const int panel_w;              // In pixels
    const int panel_h;              // ""
};

/*
 * Read the configuration file for the DIY HMD into a cJSON object,
 * for which we can use helper functions to pull out the relevant configuration data
 */
const cJSON *config_json
read_config_file(const char *file_path, const cJSON *config_json) {

    const char *file_content = u_file_read_content_from_path(file_path, NULL);
    if (file_content == NULL) {
        //TODO write an error statement to be printed via log.
        return false;
    }
    config_json = cJSON_Parse(file_content);

    free((void *)file_content);

    return config_json;
}

/*
 * With the cJSON object extracted from the config.json file we can then pull out the
 * values for each config entry. This is pretty hardcoded, but I don't see a much better
 * way at this stage. u_json_get first pulls out the
 */
void
extract_config_data(struct diy_config_data *config_data, cJSON *root) {

    size_t max_size = 50;
    u_json_get_string_into_array(u_json_get(root, "name"),   config_data->serial, max_size);
    u_json_get_string_into_array(u_json_get(root, "serial"), config_data->serial, max_size);

    u_json_get_float(u_json_get(root, "display_refresh_hz"), config_data->display_refresh_hz);

    u_json_get_double(u_json_get(root, "hFOV_deg"), config_data->hFOV_deg);
    u_json_get_double(u_json_get(root, "vFOV_deg"), config_data->vFOV_deg);
    u_json_get_double(u_json_get(root, "hCOP"),     config_data->hCOP);
    u_json_get_double(u_json_get(root, "vCOP"),     config_data->vCOP);

    u_json_get_int(u_json_get(root, "panel_w"), config_data->panel_w);
    u_json_get_int(u_json_get(root, "panel_h"), config_data->panel_h);

}





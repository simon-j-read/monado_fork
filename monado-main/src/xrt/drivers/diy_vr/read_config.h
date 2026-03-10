//
// Created by simon on 09/03/2026.
//

#ifndef XRT_READ_CONFIG_H
#define XRT_READ_CONFIG_H

const cJSON * read_config_file(const char *file_path, const cJSON *config_json);


void
extract_config_data(struct diy_config_data *config_data, cJSON *root);

#endif //XRT_READ_CONFIG_H
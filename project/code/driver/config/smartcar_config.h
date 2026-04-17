#ifndef SMARTCAR_CONFIG_H_
#define SMARTCAR_CONFIG_H_

#include <string>

bool smartcar_config_load_from_default_locations(std::string *loaded_path, std::string *error_message);

#endif

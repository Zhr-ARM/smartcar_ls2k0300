#ifndef SMARTCAR_CONFIG_H_
#define SMARTCAR_CONFIG_H_

#include <string>
#include <vector>

bool smartcar_config_load_from_default_locations(std::string *loaded_path, std::string *error_message);
bool smartcar_config_apply_toml_text(const std::string &toml_text,
                                     std::vector<std::string> *restart_required_keys,
                                     std::string *error_message);
bool smartcar_config_read_loaded_text(std::string *toml_text,
                                      std::string *loaded_path,
                                      std::string *error_message);
std::string smartcar_config_loaded_path();

#endif

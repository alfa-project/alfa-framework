/*
 * Copyright 2023 ALFA Project. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "alfa_node.hpp"

// Returns the parameter's value using its name
float AlfaNode::get_extension_parameter(string parameter_name) {
#ifdef ALFA_VERBOSE
  verbose_info("get_extension_parameter",
               "getting parameter: " + parameter_name);
#endif
  return this->get_parameter(parameter_name).get_parameter_value().get<float>();
}

// Parameters Callback ----------------------------------
rcl_interfaces::msg::SetParametersResult AlfaNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  if (configuration.hardware_support.hardware_extension) {
    result.successful = true;
    result.reason = "success";

    for (const auto &param :
         parameters)  // Go through all the changed parameters
    {
      for (std::uint16_t i = 0; i < extension_parameters.size();
           i++)  // Go through all current parameters, detect the changed
                 // one and change in memory
      {
        if (param.get_name() == extension_parameters[i].parameter_name) {
          unit_write_register(
              UNIT_USER_DEFINE_0 + i * 4,
              static_cast<int32_t>(param.as_double() * FIXED_POINT_MULTIPLIER));
        }
      }
    }

    return result;
  } else {
    verbose_not_defined("parameters_callback");
    result.successful = false;
    return result;
  }
}
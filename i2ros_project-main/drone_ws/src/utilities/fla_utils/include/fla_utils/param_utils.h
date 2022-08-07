// Copyright 2016 Massachusetts Intitute of Technology

#pragma once

#include <string>
#include <ros/ros.h>

namespace fla_utils {

template <typename T>
inline bool SafeGetParam(ros::NodeHandle & nh,
                         std::string const& param_name,
                         T & param_value) {
  if (!nh.getParam(param_name, param_value)) {
    ROS_ERROR("Failed to find parameter: %s", nh.resolveName(param_name, true).c_str());
    exit(1);
  }
  return true;
}


inline bool StringGetParam(ros::NodeHandle & nh,
                         std::string const& param_name, std::string& param_value) {
  int param_value_int;
  double param_value_double;
  bool param_value_bool;
  std::string param_value_string;
  
  if(nh.getParam(param_name, param_value_int)) {
  	param_value = std::to_string(param_value_int);
	return true;
  } else if(nh.getParam(param_name, param_value_double)) {
  	param_value = std::to_string(param_value_double);
	return true;
  } if(nh.getParam(param_name, param_value_bool)) {
  	param_value = param_value_bool ? "true" : "false";
	return true;
  } else if(nh.getParam(param_name, param_value_string)) {
  	param_value = param_value_string;
	return true;
  } else {
	ROS_ERROR("Failed to find parameter: %s", nh.resolveName(param_name, true).c_str());
	return false;
  }
}


}  // namespace fla_utils

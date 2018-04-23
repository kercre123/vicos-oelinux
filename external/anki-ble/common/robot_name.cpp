/**
 * File: robot_name.cpp
 *
 * Author: seichert
 * Created: 4/23/2018
 *
 * Description: Get/Set the Robot's name
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "robot_name.h"

#include <iostream>
#include <random>

#include <cutils/properties.h>


namespace Anki {

std::string GetRobotName() {
  char prop_value[PROPERTY_VALUE_MAX] = {0};
  (void) property_get(kRobotNamePropertyKey.c_str(), prop_value, "");

  if (prop_value[0] != '\0') {
    // The name has already been set, so return it;
    return std::string(prop_value);
  }

  (void) property_get(kProductNamePropertyKey.c_str(), prop_value, kDefaultProductName.c_str());
  if (prop_value[0] == '\0') {
    // Fatal Error.  No product name.
    std::cerr << "Fatal: No product name found" << std::endl;
    return std::string("");
  }

  std::string product_name(prop_value);
  // The robot shall be named "<ProductName> XYXY" where
  // X is a random letter and Y is a random digit.
  // In some fonts, '0' and 'O' look alike, so they are excluded.
  // In addition, we exclude '1', 'I', and 'L' for the same reason.

  std::vector<char> letters = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J',
                               'K', 'M', 'N', 'P', 'R', 'S', 'T', 'U', 'V',
                               'W', 'X', 'Y', 'Z'};
  std::vector<char> digits =  {'1', '2', '3', '4', '5', '6', '7', '8', '9'};


  std::random_device rd;
  std::mt19937 gen(rd());

  // 198 = 22 * 9.  This should allow for a uniform distribution of values
  // when we select from either the letters or digits vectors.
  std::uniform_int_distribution<> dis(1, 198);

  std::string robot_id;
  for (int i = 0 ; i < 2 ; i++) {
    char letter = letters[dis(gen) % letters.size()];
    robot_id.push_back(letter);
    char digit = digits[dis(gen) % digits.size()];
    robot_id.push_back(digit);
  }

  std::string robot_name = product_name + " " + robot_id;
  int rc = property_set(kRobotNamePropertyKey.c_str(), robot_name.c_str());
  if (rc) {
    std::cerr << "Fatal: Failed to set robot's name in property" << std::endl;
    return std::string("");
  }

  std::cout << "Robot has been christened \"" << robot_name << "\"" << std::endl;
  return robot_name;
}

} // namespace Anki

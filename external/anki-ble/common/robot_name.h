/**
 * File: robot_name.h
 *
 * Author: seichert
 * Created: 4/23/2018
 *
 * Description: Get/Set the Robot's name
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include <string>

namespace Anki {

const std::string kRobotNamePropertyKey("anki.robot.name");
const std::string kProductNamePropertyKey("ro.anki.product.name");
const std::string kDefaultProductName("Vector");

std::string GetRobotName();

} // namespace Anki

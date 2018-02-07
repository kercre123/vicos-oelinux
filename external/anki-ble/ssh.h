/**
 * File: ssh.h
 *
 * Author: seichert
 * Created: 1/22/2018
 *
 * Description: Routines for managing SSH
 *
 * Copyright: Anki, Inc. 2018
 *
 **/


#pragma once

namespace Anki {
int SetSSHAuthorizedKeys(const std::string& keys);
} // namespace Anki

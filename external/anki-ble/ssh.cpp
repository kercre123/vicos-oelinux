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

#include "fileutils.h"

namespace Anki {

static std::string GetPathToSSHAuthorizedKeys()
{
  return "/data/root/.ssh/authorized_keys";
}

int SetSSHAuthorizedKeys(const std::string& keys)
{
  int rc = WriteFileAtomically(GetPathToSSHAuthorizedKeys(), keys, (S_IRUSR | S_IWUSR));
  return rc;
}

} // namespace Anki

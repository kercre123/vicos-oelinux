/**
 * File: keymaster.h
 *
 * Author: Stuart Eichert
 * Created: 9/7/2018
 *
 * Description: Get/Use TrustZone crypto keys
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#ifndef __KEYMASTER_H__
#define __KEYMASTER_H__

#include <cstdint>
#include <vector>

class KeyMaster {
 public:
  KeyMaster();
  ~KeyMaster();
  int GenerateKeyBlob(std::vector<uint8_t>& keyBlob);
  int SignData(const std::vector<uint8_t>& keyBlob,
               const std::vector<uint8_t>& data,
               std::vector<uint8_t>& signature);
 private:
  int Initialize();
  void* keymaster_handle;
};


#endif /* __KEYMASTER_H__ */

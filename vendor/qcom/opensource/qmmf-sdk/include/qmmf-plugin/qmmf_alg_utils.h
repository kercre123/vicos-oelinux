/*
* Copyright (c) 2017, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <dlfcn.h>
#include <stdint.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

#ifdef ANDROID

#include <cutils/properties.h>
#include <utils/Log.h>

#else

#define ALOGE(...) ((void)fprintf(stderr, __VA_ARGS__))
#define ALOGW(...) ((void)fprintf(stdout, __VA_ARGS__))
#define ALOGI(...) ((void)fprintf(stdout, __VA_ARGS__))
#define ALOGD(...) ((void)fprintf(stdout, __VA_ARGS__))

#endif

namespace qmmf {

namespace qmmf_alg_plugin {

/** Utils:
 *
 *  This class defines the helper utils in qmmf framework
 **/
class Utils {
 public:
  /** ThrowException
   *    @title: exception title
   *    @msg: exception message
   *
   * Throws exception
   *
   * return: void
   **/
  static void ThrowException(std::string title, std::string msg) {
    std::stringstream s;
    s << "Error: " << title << " " << msg;
    throw std::runtime_error(s.str());
  }

  /** GetDataFolder
   *
   * Returns data folder
   *
   * return: data folder
   **/
  static std::string GetDataFolder() {
    return "/data/misc/qmmf/";
  }

  /** GetAlgLibFolder
   *
   * Returns QMMF algorithm libraries folder
   *
   * return: algorithm libraries folder
   **/
  static std::string GetAlgLibFolder() {
#ifdef ANDROID_LIBPATH
    return "/vendor/lib/qmmf/alg-plugins/";
#else
    return "/usr/lib/qmmf/alg-plugins/";
#endif
  }

  /** MakeDivisibleBy
   *    @val: value
   *    @div: divider
   *
   * Calculates closest greater value which is divisible by divider
   *
   * return: closest greater value which is divisible by divider
   **/
  static size_t MakeDivisibleBy(size_t val, uint32_t div) {
    return div > 0 ? ((val + div - 1) / div) * div : val;
  }

  /** GetWidthInBytes
   *    @val: value
   *    @pixel_size: pixel size
   *
   * Calculates width in bytes
   *
   * return: width in bytes
   **/
  static uint32_t GetWidthInBytes(uint32_t val, float pixel_size) {
    return (uint32_t)std::ceil(val * pixel_size);
  }

  /** GCD
   *    @a: value a
   *    @a: value b
   *
   * Calculates gcd of the values
   *
   * return: gcd of the values
   **/
  static uint32_t GCD(uint32_t a, uint32_t b) {
    // TODO uncomment when cpp 17 is enabled
    // return std::gcd(a, b);

    if (0 == a) {
      return b;
    }
    if (0 == b) {
      return a;
    }

    while (a != b) {
      if (a > b) {
        a -= b;
      } else {
        b -= a;
      }
    }

    return a;
  }

  /** LCM
   *    @a: value a
   *    @a: value b
   *
   * Calculates lcm of the values
   *
   * return: lcm of the values
   **/
  static uint32_t LCM(uint32_t a, uint32_t b) {
    // TODO uncomment when cpp 17 is enabled
    // return std::lcm(a, b);

    if (0 == a) {
      return b;
    }
    if (0 == b) {
      return a;
    }

    return ((uint64_t)a * b) / GCD(a, b);
  }

  /** GetProperty
   *    @property: property
   *    @default_value: default value
   *
   * Gets requested property value
   *
   * return: property value
   **/
  template <typename TProperty>
  static TProperty GetProperty(std::string property, TProperty default_value) {
    TProperty value = default_value;
#ifdef ANDROID
    char prop_val[PROPERTY_VALUE_MAX];
    std::stringstream s;
    s << default_value;
    property_get(property.c_str(), prop_val, s.str().c_str());

    std::stringstream output(prop_val);
    output >> value;
#else
    try {
      std::string property_file = QMMF_PROPERTIES_DIRECTORY + property;
      std::ifstream property_file_handler(property_file);
      property_file_handler >> value;
    } catch (...) {
      // Do nothing since the property doesn't exist
    }
#endif
    return value;
  }

  /** SetProperty
   *    @property: property
   *    @value: value
   *
   * Sets requested property value
   *
   * return: nothing
   **/
  template <typename TProperty>
  static void SetProperty(std::string property, TProperty value) {
    std::stringstream s;
    s << value;
    std::string value_string = s.str();
#ifdef ANDROID
    value_string.resize(PROPERTY_VALUE_MAX);
    property_set(property.c_str(), value_string.c_str());
#else
    std::string property_file = QMMF_PROPERTIES_DIRECTORY + property;
    std::ofstream property_file_handler(property_file);
    property_file_handler << value_string;
#endif
  }

  /** LoadLib
   *    @lib_name: name of the library
   *    @lib_handle (output): loaded lib handle
   *
   * Loads library
   *
   * return: lib handle
   **/
  static void LoadLib(std::string lib_name, void *&lib_handle) {
    dlerror();

#ifndef ANDROID
    lib_name = QMMF_DLL_PATH + lib_name;
#endif

    lib_handle = dlopen(lib_name.c_str(), RTLD_NOW);
    const char *dlsym_error = dlerror();
    if (!lib_handle || dlsym_error) {
      ThrowException(__func__, dlsym_error);
    }
  }

  /** UnloadLib
   *    @lib_handle: lib handle
   *
   * Unloads library
   *
   * return: nothing
   **/
  static void UnloadLib(void *&lib_handle) {
    if (nullptr != lib_handle) {
      dlclose(lib_handle);
      lib_handle = nullptr;
    }
  }

  /** LoadLibHandler
   *    @lib_handle: lib handle
   *    @handler_name: name of the handler
   *    @handler (output): loaded handler
   *
   * Loads lib handler
   *
   * return: nothing
   **/
  template <typename THandler>
  static void LoadLibHandler(void *lib_handle, std::string handler_name,
                             THandler &handler) {
    handler = (THandler)dlsym(lib_handle, handler_name.c_str());
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
      ThrowException(__func__, dlsym_error);
    }
  }
};

}; // namespace qmmf_alg_plugin

}; // namespace qmmf

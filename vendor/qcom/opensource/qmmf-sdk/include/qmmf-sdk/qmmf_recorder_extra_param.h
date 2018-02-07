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

#include <cstddef>
#include <cerrno>
#include <cstdint>
#include <memory>
#include <cstring>
#include <type_traits>
#include <vector>
#include <map>

#include <utils/Log.h>

namespace qmmf {

namespace recorder {

/**
 * Base structure for all data structures.
 */
struct DataTagBase {
 private:
  uint32_t tag_id;

 public:
  DataTagBase(const uint32_t &tag) : tag_id(tag) {}
  bool IsValidTag(const uint32_t &tag) {
    return (tag_id == tag) ? true : false;
  }
};

class ExtraParam {
 public:
  ExtraParam();
  ExtraParam(const void *data, const size_t &size);
  ExtraParam(const ExtraParam &other);

  ~ExtraParam();

  ExtraParam &operator=(const ExtraParam &other);

  /**
   * Remove all data from the container.
   */
  int32_t Clear();

  /**
   * Check if the container is empty
   */
  bool IsEmpty() const;

  /**
   * Check if the container has any data for the given tag.
   */
  bool Exists(uint32_t tag) const;

  /**
   * Total count of the tags recorded in the container.
   */
  size_t TagCount() const;

  /**
   * Number entries of the specified data tag in the container.
   */
  size_t EntryCount(uint32_t tag) const;

  /**
   * Add the data structure associated with the tag ID at the the given
   * entry position. The default entry position is the first(0).
   * If a tag entry with this number already exists in the container its data
   * structure will be overwritten with the new one.
   * It is required for the entry numbers to be subsequent.
   */
  template<typename T>
  int32_t Update(uint32_t tag, const T &data, uint32_t entry = 0) {

    static_assert(std::is_base_of<DataTagBase, T>::value,
        "The data type is not derived from base struct DataTagBase!");

    DataTagBase base_data = static_cast<DataTagBase>(data);
    if (!base_data.IsValidTag(tag)) {
      ALOGE("%s: This data does not belong to tag %d!", __func__, tag);
      return -EINVAL;
    }

    auto ret = FillDataEntry(tag, entry, data);
    if (0 != ret) {
      ALOGE("%s: Failed to fill data entry!", __func__);
      return ret;
    }
    return 0;
  }

  /**
   * Retrieve the data structure associated with the tag ID at the the given
   * entry position. The default entry position is the first(0).
   */
  template<typename T>
  int32_t Fetch(uint32_t tag, T &data, uint32_t entry = 0) const {

    static_assert(std::is_base_of<DataTagBase, T>::value,
        "The data type is not derived from base struct DataTagBase!");

    DataTagBase base_data = static_cast<DataTagBase>(data);
    if (!base_data.IsValidTag(tag)) {
      ALOGE("%s: This data does not belong to tag %d!", __func__, tag);
      return -EINVAL;
    }

    auto ret = FetchDataEntry(tag, entry, data);
    if (0 != ret) {
      ALOGE("%s: Failed to add data entry!", __func__);
      return ret;
    }
    return 0;
  }

  /**
   * Remove a data tag entry from the container.
   */
  int32_t Remove(uint32_t tag, uint32_t entry);

  /**
   * Completely erase a tag and all of its entries from the container.
   */
  int32_t Erase(uint32_t tag);

  /**
   * Acquire a raw memory block from the caller.
   * If ExtraParam already contains tag data, it is replaced with the new data.
   */
  int32_t Acquire(const void *data, const size_t &size);

  /**
   * Give raw memory pool ownership to the caller.
   * The new owner is responsible to free the allocated memory via delete.
   */
  std::shared_ptr<void> ReleaseOwnership();

  /**
   * Temporally take reference pointer to the raw memory pool.
   * Until the reference is returned via ReturnAndUnlock() write/delete
   * operations from the container are forbidden.
   */
  const void* GetAndLock() const;

  /**
   * Return the reference pointer acquired via GetAndLock() and unlock the
   * internal raw memory pool for write/delete operations. After calling this
   * operation the reference pointer is no longer valid.
   */
  int32_t ReturnAndUnlock(const void *data) const;

  /**
   * Retrieve internal raw memory pool size in bytes.
   */
  size_t Size() const;

 private:
  typedef uint8_t byte_t;

  /**
   * Descriptor of a tag data structure in the raw memory pool.
   * It is attached to the beginning of each structure recorded in the memory.
   *
   * @tag_id: Unique ID describing the data structure type.
   * @entry_id: Subsequent number of the entry in the same tag_id.
   * @data_size: Size of the actual data structure. The data is located at an
   *             address with offset kDataDescSize from the beginning of the
   *             DataDecriptor.
   */
  struct DataDecriptor {
    uint32_t tag_id;
    uint32_t entry_id;
    uint32_t data_size;
  };

  template<typename T>
  int32_t FillDataEntry(uint32_t &tag, uint32_t &entry, const T &data) {

    if (locked_) {
      ALOGE("%s: Can't add tag entry to a locked Container!", __func__);
      return -EPERM;
    }

    uint32_t data_size = sizeof(T);
    uint32_t entry_size = kDataDescSize + data_size;
    uintptr_t entry_offset = 0;

    auto it = data_map_.find(tag);
    if (it != data_map_.end() && it->second.size() > entry) {
      // Tag and entry exist, fetch the memory offset.
      entry_offset = data_map_.at(tag).at(entry);
    } else {
      // Tag may exists but entry definitely does not.
      entry_offset = data_buffer_.size();

      if (it == data_map_.end()) {
        // Neither tag nor entry exist in the map, add them.
        if (0 != entry) {
          ALOGE("%s: Invalid entry number! Entries must be subsequent!",
              __func__);
          return -EINVAL;
        }
        std::vector<uintptr_t> v(1, entry_offset);
        data_map_.emplace(tag, v);
      } else if (it != data_map_.end() && it->second.size() <= entry) {
        // Tag exist in the map, but the entry does not.
        if ((it->second.size() - entry) > 1) {
          ALOGE("%s: Invalid entry number! Entries must be subsequent!",
              __func__);
          return -EINVAL;
        }
        data_map_.at(tag).emplace_back(entry_offset);
      }

      data_buffer_.insert(data_buffer_.end(), entry_size, 0);
      // Fill the data descriptor of the new entry.
      DataDecriptor *desc =
          reinterpret_cast<DataDecriptor*>(data_buffer_.data() + entry_offset);
      desc->tag_id    = tag;
      desc->entry_id  = entry;
      desc->data_size = data_size;
    }

    // Copy the data into the buffer.
    void *dest = reinterpret_cast<void*>(
        data_buffer_.data() + entry_offset + kDataDescSize);
    const void *src = reinterpret_cast<const void*>(&data);
    memcpy(dest, src, data_size);

    return 0;
  }

  template<typename T>
  int32_t FetchDataEntry(uint32_t &tag, uint32_t &entry, T &data) const {

    uint32_t data_size = sizeof(T);
    uintptr_t entry_offset = data_map_.at(tag).at(entry);

    void *dest = reinterpret_cast<void*>(&data);
    const void *src = reinterpret_cast<const void*>(
        data_buffer_.data() + entry_offset + kDataDescSize);
    memcpy(dest, src, data_size);

    return 0;
  }

  int32_t RemoveDataEntry(uint32_t &tag, uint32_t &entry);

  int32_t EraseDataTag(uint32_t &tag);

  int32_t ReorganizeDataMap(uintptr_t entry_offset);

  // Lock meant for preventing write/delete operations.
  mutable bool locked_;

  // Tag map with memory offset entries where the actual data is stored.
  std::map<uint32_t, std::vector<uintptr_t> > data_map_;
  std::vector<byte_t> data_buffer_;

  static const size_t kDataDescSize = sizeof(DataDecriptor);

};

class VideoExtraParam : public ExtraParam {
 public:
  VideoExtraParam()
      : ExtraParam() {};

  VideoExtraParam(const void *data, const size_t &size)
      : ExtraParam(data, size) {};

  VideoExtraParam(const VideoExtraParam &other)
      : ExtraParam(static_cast<ExtraParam>(other)) {};

};

class ImageConfigParam : public ExtraParam {
 public:
  ImageConfigParam()
      : ExtraParam() {};

  ImageConfigParam(const void *data, const size_t &size)
      : ExtraParam(data, size) {};

  ImageConfigParam(const ImageConfigParam &other)
  : ExtraParam(static_cast<ExtraParam>(other)) {};

};

}; //namespace recorder.

}; //namespace qmmf.

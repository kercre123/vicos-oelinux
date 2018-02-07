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

#include <algorithm>

#include "qmmf-sdk/qmmf_recorder_extra_param.h"

namespace qmmf {

namespace recorder {

ExtraParam::ExtraParam()
    : locked_(false) {

}

ExtraParam::ExtraParam(const void *data, const size_t &size)
    : locked_(false) {

  Acquire(data, size);
}

ExtraParam::ExtraParam(const ExtraParam &other)
    : locked_(false) {

  const void *source_data = other.GetAndLock();
  Acquire(source_data, other.Size());
  other.ReturnAndUnlock(source_data);
}

ExtraParam::~ExtraParam() {

  Clear();
}

ExtraParam &ExtraParam::operator=(const ExtraParam &other) {

  const void *source_data = other.GetAndLock();
  if (source_data != reinterpret_cast<const void*>(data_buffer_.data())) {
    Acquire(source_data, other.Size());
  }
  other.ReturnAndUnlock(source_data);

  return *this;
}

int32_t ExtraParam::Clear() {

  if (locked_) {
    ALOGE("%s: Can't clear a locked Container!", __func__);
    return -EPERM;
  }
  data_map_.clear();
  data_buffer_.clear();

  return 0;
}

bool ExtraParam::IsEmpty() const {

  return data_map_.empty();
}

bool ExtraParam::Exists(uint32_t tag) const {

  return (data_map_.find(tag) != data_map_.end());
}

size_t ExtraParam::TagCount() const {

  return data_map_.size();
}

size_t ExtraParam::EntryCount(uint32_t tag) const {

  if (!Exists(tag)) return 0;
  return data_map_.at(tag).size();
}

int32_t ExtraParam::Remove(uint32_t tag, uint32_t entry) {

  auto ret = RemoveDataEntry(tag, entry);
  if (0 != ret) {
    ALOGE("%s: Failed to remove tag %u, entry %u!", __func__, tag, entry);
    return ret;
  }
  return 0;
}

int32_t ExtraParam::Erase(uint32_t tag) {

  auto ret = EraseDataTag(tag);
  if (0 != ret) {
    ALOGE("%s: Failed to erase tag %u!", __func__, tag);
    return ret;
  }
  return 0;
}

int32_t ExtraParam::Acquire(const void *data, const size_t &size) {

  if (locked_) {
    ALOGE("%s: Assignment to a locked Container!", __func__);
    return -EPERM;
  }

  data_map_.clear();
  data_buffer_.clear();

  data_buffer_.resize(size);
  memcpy(data_buffer_.data(), data, size);

  uintptr_t entry_offset = 0;
  while (data_buffer_.size() > entry_offset) {
    DataDecriptor *desc =
        reinterpret_cast<DataDecriptor*>(data_buffer_.data() + entry_offset);

    auto it = data_map_.find(desc->tag_id);
    if (it == data_map_.end()) {
      // Neither tag nor entry exist in the map, add them.
      std::vector<uintptr_t> entry(1, entry_offset);
      data_map_.emplace(desc->tag_id, entry);
    } else if (it != data_map_.end() && it->second.size() <= desc->entry_id) {
      // Tag exist in the map, but the entry does not.
      data_map_.at(desc->tag_id).emplace_back(entry_offset);
    }

    // Increment the memory offset with the total entry size.
    entry_offset += desc->data_size + kDataDescSize;
  }

  return 0;
}

std::shared_ptr<void> ExtraParam::ReleaseOwnership() {

  auto size = data_buffer_.size();
  std::shared_ptr<void> data(new byte_t[size]);
  memcpy(data.get(), data_buffer_.data(), size);

  data_map_.clear();
  data_buffer_.clear();
  return data;
}

const void* ExtraParam::GetAndLock() const {

  locked_ = true;
  return reinterpret_cast<const void*>(data_buffer_.data());
}

int32_t ExtraParam::ReturnAndUnlock(const void *data) const {

  if (!locked_) {
    ALOGE("%s: Can't unlock a non-locked Container!", __func__);
    return -EPERM;
  }
  if (data != reinterpret_cast<const void*>(data_buffer_.data())) {
    ALOGE("%s: Can't unlock Container with wrong pointer!", __func__);
    return -EINVAL;
  }
  locked_ = false;
  return 0;
}

size_t ExtraParam::Size() const {

  return data_buffer_.size();
}

int32_t ExtraParam::RemoveDataEntry(uint32_t &tag, uint32_t &entry) {

  if (locked_) {
    ALOGE("%s: Can't remove entry from a locked Container!", __func__);
    return -EPERM;
  }

  auto it = data_map_.find(tag);
  if (it == data_map_.end()) {
    ALOGE("%s: Tag %u does not exist!", __func__, tag);
    return -EINVAL;
  } else if (it->second.size() <= entry) {
    ALOGE("%s: Entry %u does not exist!", __func__, entry);
    return -EINVAL;
  }

  // Retrieve the memory offset to the actual data of the entry.
  auto entry_offset = it->second.at(entry);

  // Iterator to the beginning of the memory buffer.
  auto const& mbegin = data_buffer_.begin();

  DataDecriptor *desc =
      reinterpret_cast<DataDecriptor*>(data_buffer_.data() + entry_offset);

  // Add the size of the descriptor to the total size of the entry.
  uint32_t entry_size = desc->data_size + kDataDescSize;

  // Erase the data from the buffer.
  data_buffer_.erase(mbegin + entry_offset, mbegin + entry_size);

  auto ret = ReorganizeDataMap(entry_offset);
  if (0 != ret) {
    ALOGE("%s: Failed to reorganize data mapping!", __func__);
    return ret;
  }

  it->second.erase(it->second.begin() + entry);
  return 0;
}

int32_t ExtraParam::EraseDataTag(uint32_t &tag) {

  if (locked_) {
    ALOGE("%s: Can't erase tag from a locked Container!", __func__);
    return -EPERM;
  }

  if (data_map_.find(tag) == data_map_.end()) {
    ALOGE("%s: Tag %u does not exist!", __func__, tag);
    return -EINVAL;
  }

  // Sort memory offsets in descending order in order to simplify
  // the erasing procedure.
  auto &entries = data_map_.at(tag);
  std::sort(entries.begin(), entries.end(), std::greater<uintptr_t>());

  // Iterator to the beginning of the memory buffer.
  auto const& mbegin = data_buffer_.begin();

  // Iterate over the memory offsets for each entry.
  for (auto const& entry_offset : entries) {
    DataDecriptor *desc =
        reinterpret_cast<DataDecriptor*>(data_buffer_.data() + entry_offset);

    // Add the size of the descriptor to the total size of the entry.
    uint32_t entry_size = desc->data_size + kDataDescSize;

    // Erase the data from the buffer.
    data_buffer_.erase(mbegin + entry_offset, mbegin + entry_size);
  }
  // Get the lowest memory offset from which data was erased.
  uintptr_t memory_offset = entries.back();

  auto ret = ReorganizeDataMap(memory_offset);
  if (0 != ret) {
    ALOGE("%s: Failed to reorganize data mapping!", __func__);
    return ret;
  }

  entries.clear();
  data_map_.erase(tag);
  return 0;
}

int32_t ExtraParam::ReorganizeDataMap(uintptr_t entry_offset) {

  // Retrieve the data descriptor and update the data map entry.
  while (data_buffer_.size() > entry_offset) {
    DataDecriptor *desc =
        reinterpret_cast<DataDecriptor*>(data_buffer_.data() + entry_offset);

    // TODO: Revisit! Think this is not needed as all entries left in
    // data_buffer_ should be in data_map_.
    auto it = data_map_.find(desc->tag_id);
    if (it == data_map_.end()) {
      ALOGE("%s: Tag %u does not exist!", __func__, desc->tag_id);
      return -EFAULT;
    } else if (it->second.size() <= desc->entry_id) {
      ALOGE("%s: Entry %u does not exist!", __func__, desc->entry_id);
      return -EFAULT;
    }
    data_map_.at(desc->tag_id).at(desc->entry_id) = entry_offset;

    // Increment the memory offset with the total entry size.
    entry_offset += desc->data_size + kDataDescSize;
  }

  return 0;
}

}; //namespace recorder.

}; //namespace qmmf.

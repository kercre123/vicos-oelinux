/**
 * File: byte_vector.cpp
 *
 * Author: seichert
 * Created: 2/28/2018
 *
 * Description: ByteVector class
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include "byte_vector.h"

namespace Anki {
ByteVector::ByteVector() {
  v = new std::vector<uint8_t>();
}
ByteVector::~ByteVector() {
  delete v; v = nullptr;
}

ByteVector::ByteVector(const ByteVector& other) {
  v = new std::vector<uint8_t>(*(other.v));
}

ByteVector::ByteVector(ByteVector&& other) {
  v = other.v;
  other.v = nullptr;
}

ByteVector::ByteVector(size_type count, const uint8_t& value) {
  v = new std::vector<uint8_t>(count, value);
}

ByteVector::ByteVector(size_type count) {
  v = new std::vector<uint8_t>(count);
}

ByteVector::ByteVector(const std::vector<uint8_t>& value) {
  v = new std::vector<uint8_t>(value);
}
ByteVector::ByteVector(value_type* begin, value_type* end) {
  v = new std::vector<uint8_t>(begin, end);
}

ByteVector::ByteVector(std::initializer_list<value_type> il) {
  v = new std::vector<uint8_t>(il);
}

ByteVector& ByteVector::operator=(const ByteVector& other ) {
  *v = *(other.v);
  return *this;
}

void ByteVector::push_back(const std::vector<uint8_t>& o, size_type offset, size_type length) {
  if (length == npos) {
    length = o.size() - offset;
  }
  std::copy(o.begin() + offset, o.begin() + offset + length, std::back_inserter(*v));
}

void ByteVector::push_back(value_type* b, size_type length) {
  std::vector<uint8_t> o(b, b + length);
  push_back(o);
}

void ByteVector::push_back_le(const uint16_t i) {
  v->push_back((uint8_t) (i & 0xff));
  v->push_back((uint8_t) ((i >> 8) & 0xff));
}

void ByteVector::push_back_le(const int16_t i) {
  v->push_back((uint8_t) (i & 0xff));
  v->push_back((uint8_t) ((i >> 8) & 0xff));
}

void ByteVector::push_back_le(const uint32_t i) {
  v->push_back((uint8_t) (i & 0xff));
  v->push_back((uint8_t) ((i >> 8) & 0xff));
  v->push_back((uint8_t) ((i >> 16) & 0xff));
  v->push_back((uint8_t) ((i >> 24) & 0xff));
}

void ByteVector::push_back_le(const int32_t i) {
  v->push_back((uint8_t) (i & 0xff));
  v->push_back((uint8_t) ((i >> 8) & 0xff));
  v->push_back((uint8_t) ((i >> 16) & 0xff));
  v->push_back((uint8_t) ((i >> 24) & 0xff));
}

void ByteVector::push_back_le(const uint64_t i) {
  v->push_back((uint8_t) (i & 0xff));
  v->push_back((uint8_t) ((i >> 8) & 0xff));
  v->push_back((uint8_t) ((i >> 16) & 0xff));
  v->push_back((uint8_t) ((i >> 24) & 0xff));
  v->push_back((uint8_t) ((i >> 32) & 0xff));
  v->push_back((uint8_t) ((i >> 40) & 0xff));
  v->push_back((uint8_t) ((i >> 48) & 0xff));
  v->push_back((uint8_t) ((i >> 56) & 0xff));
}

void ByteVector::push_back_le(const int64_t i) {
  v->push_back((uint8_t) (i & 0xff));
  v->push_back((uint8_t) ((i >> 8) & 0xff));
  v->push_back((uint8_t) ((i >> 16) & 0xff));
  v->push_back((uint8_t) ((i >> 24) & 0xff));
  v->push_back((uint8_t) ((i >> 32) & 0xff));
  v->push_back((uint8_t) ((i >> 40) & 0xff));
  v->push_back((uint8_t) ((i >> 48) & 0xff));
  v->push_back((uint8_t) ((i >> 56) & 0xff));
}

void ByteVector::push_back(const std::string& s, bool include_terminator) {
  std::copy(s.begin(), s.end(), std::back_inserter(*v));
  if (include_terminator) {
    v->push_back((uint8_t) 0);
  }
}

uint16_t ByteVector::read_uint16_le(size_type offset) const {
  uint16_t i = read_uint16_le(*v, offset);
  return i;
}

int16_t ByteVector::read_int16_le(size_type offset) const {
  int16_t i = read_int16_le(*v, offset);
  return i;
}

uint32_t ByteVector::read_uint32_le(size_type offset) const {
  uint32_t i = read_uint32_le(*v, offset);
  return i;
}

int32_t ByteVector::read_int32_le(size_type offset) const {
  int32_t i = read_int32_le(*v, offset);
  return i;
}

uint64_t ByteVector::read_uint64_le(size_type offset) const {
  uint64_t i = read_uint64_le(*v, offset);
  return i;
}

int64_t ByteVector::read_int64_le(size_type offset) const {
  int64_t i = read_int64_le(*v, offset);
  return i;
}

uint16_t ByteVector::read_uint16_le(const std::vector<uint8_t>& vec, size_type offset) {
  uint16_t i = ( ((uint16_t) vec[offset+1] << 8) |
                 ((uint16_t) vec[offset]) );
  return i;
}

int16_t ByteVector::read_int16_le(const std::vector<uint8_t>& vec, size_type offset) {
  int16_t i = ( ((int16_t) vec[offset+1] << 8) |
                ((int16_t) vec[offset]) );
  return i;
}

uint32_t ByteVector::read_uint32_le(const std::vector<uint8_t>& vec, size_type offset) {
  uint32_t i = ( ((uint32_t) vec[offset+3] << 24) |
                 ((uint32_t) vec[offset+2] << 16) |
                 ((uint32_t) vec[offset+1] << 8) |
                 ((uint32_t) vec[offset]) );
  return i;
}

int32_t ByteVector::read_int32_le(const std::vector<uint8_t>& vec, size_type offset) {
  int32_t i = ( ((int32_t) vec[offset+3] << 24) |
                ((int32_t) vec[offset+2] << 16) |
                ((int32_t) vec[offset+1] << 8) |
                ((int32_t) vec[offset]) );
  return i;
}

uint64_t ByteVector::read_uint64_le(const std::vector<uint8_t>& vec, size_type offset) {
  uint64_t i = ( ((uint64_t) vec[offset+7] << 56) |
                 ((uint64_t) vec[offset+6] << 48) |
                 ((uint64_t) vec[offset+5] << 40) |
                 ((uint64_t) vec[offset+4] << 32) |
                 ((uint64_t) vec[offset+3] << 24) |
                 ((uint64_t) vec[offset+2] << 16) |
                 ((uint64_t) vec[offset+1] << 8) |
                 ((uint64_t) vec[offset]) );
  return i;
}

int64_t ByteVector::read_int64_le(const std::vector<uint8_t>& vec, size_type offset) {
  int64_t i = ( ((int64_t) vec[offset+7] << 56) |
                ((int64_t) vec[offset+6] << 48) |
                ((int64_t) vec[offset+5] << 40) |
                ((int64_t) vec[offset+4] << 32) |
                ((int64_t) vec[offset+3] << 24) |
                ((int64_t) vec[offset+2] << 16) |
                ((int64_t) vec[offset+1] << 8) |
                ((int64_t) vec[offset]) );
  return i;
}

} // namespace Anki

/**
 * File: byte_vector.h
 *
 * Author: seichert
 * Created: 2/28/2018
 *
 * Description: ByteVector class
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace Anki {
class ByteVector {
public:
  typedef uint8_t                                    value_type;
  typedef std::allocator<uint8_t>                    allocator_type;
  typedef typename allocator_type::reference         reference;
  typedef typename allocator_type::const_reference   const_reference;
  typedef std::vector<uint8_t>::iterator             iterator;
  typedef std::vector<uint8_t>::const_iterator       const_iterator;
  typedef typename allocator_type::size_type         size_type;
  typedef typename allocator_type::difference_type   difference_type;
  typedef typename allocator_type::pointer           pointer;
  typedef typename allocator_type::const_pointer     const_pointer;
  typedef std::reverse_iterator<iterator>            reverse_iterator;
  typedef std::reverse_iterator<const_iterator>      const_reverse_iterator;

  static const size_type npos = -1;

  ByteVector();
  ByteVector(const ByteVector& other);
  ByteVector(ByteVector&& other);
  explicit ByteVector(size_type count, const uint8_t& value);
  explicit ByteVector(size_type count);
  explicit ByteVector(const std::vector<uint8_t>& value);
  ByteVector(value_type* begin, value_type* end);
  ByteVector(std::initializer_list<value_type> il);
  ~ByteVector();
  ByteVector& operator=(const ByteVector& other );

  void assign(size_type count, const uint8_t& value) { v->assign(count, value); }
  void assign(std::initializer_list<value_type> il) { v->assign(il); }

  iterator begin() noexcept { return v->begin(); }
  const_iterator begin() const noexcept { return v->begin(); }
  iterator end() noexcept { return v->end(); }
  const_iterator end() const noexcept { return v->end(); }

  reverse_iterator rbegin() noexcept { return v->rbegin(); }
  const_reverse_iterator rbegin() const noexcept { return v->rbegin(); }
  reverse_iterator rend() noexcept { return v->rend(); }
  const_reverse_iterator rend() const noexcept { return v->rend(); }

  const_iterator cbegin() const noexcept { return v->cbegin(); }
  const_iterator cend() const noexcept { return v->cend(); }
  const_reverse_iterator crbegin() const noexcept { return v->crbegin(); }
  const_reverse_iterator crend() const noexcept { return v->crend(); }

  size_type size() const noexcept { return v->size(); }
  size_type max_size() const noexcept { return v->max_size(); }
  size_type capacity() const noexcept { return v->capacity(); }
  bool empty() const noexcept { return v->empty(); }
  void reserve(size_type n) { return v->reserve(n); }
  void shrink_to_fit() noexcept { return v->shrink_to_fit(); }

  reference operator[](size_type n) { return (*v)[n]; }
  const_reference operator[](size_type n) const { return (*v)[n]; }
  reference at(size_type n) { return v->at(n); }
  const_reference at(size_type n) const { return v->at(n); }

  reference front() { return v->front(); }
  const_reference front() const { return v->front(); }
  reference back() { return v->back(); }
  const_reference back() const { return v->back(); }

  value_type* data() noexcept { return v->data(); }
  const value_type* data() const noexcept { return v->data(); }

  void push_back(const value_type& x) { return v->push_back(x); }
  void push_back(value_type&& x) { return v->push_back(x); }
  void pop_back() { v->pop_back(); }

  iterator insert(const_iterator position, const value_type& x) { return v->insert(position, x); }
  iterator insert(const_iterator position, value_type&& x) { return v->insert(position, x); }
  iterator insert(const_iterator position, size_type n, const value_type& x) {
    return v->insert(position, n, x);
  }
  template <class InputIterator>
  iterator insert(const_iterator position, InputIterator first, InputIterator last) {
    return v->insert(position, first, last);
  }
  iterator insert(const_iterator position, std::initializer_list<value_type> il) {
    return v->insert(position, il);
  }

  iterator erase(const_iterator position) {
    return v->erase(position);
  }
  iterator erase(const_iterator first, const_iterator last) {
    return v->erase(first, last);
  }

  void clear() noexcept { v->clear(); }

  void resize(size_type sz) { v->resize(sz); }
  void resize(size_type sz, const value_type& c) { v->resize(sz,c); }

  void swap(ByteVector& b) { v->swap(*(b.v)); }

  void push_back(const std::vector<uint8_t>& o, size_type offset = 0, size_type length = npos);
  void push_back(const ByteVector& b, size_type offset = 0, size_type length = npos) {
    push_back(*(b.v), offset, length);
  }
  void push_back(value_type* b, size_type length);

  void push_back_le(const uint16_t i);
  void push_back_le(const int16_t i);
  void push_back_le(const uint32_t i);
  void push_back_le(const int32_t i);
  void push_back_le(const uint64_t i);
  void push_back_le(const int64_t i);
  void push_back(const std::string& s, bool include_terminator = false);

  uint16_t read_uint16_le(size_type offset = 0) const;
  int16_t read_int16_le(size_type offset = 0) const;
  uint32_t read_uint32_le(size_type offset = 0) const;
  int32_t read_int32_le(size_type offset = 0) const;
  uint64_t read_uint64_le(size_type offset = 0) const;
  int64_t read_int64_le(size_type offset = 0) const;

  static uint16_t read_uint16_le(const std::vector<uint8_t>& vec, size_type offset = 0);
  static int16_t read_int16_le(const std::vector<uint8_t>& vec, size_type offset = 0);
  static uint32_t read_uint32_le(const std::vector<uint8_t>& vec, size_type offset = 0);
  static int32_t read_int32_le(const std::vector<uint8_t>& vec, size_type offset = 0);
  static uint64_t read_uint64_le(const std::vector<uint8_t>& vec, size_type offset = 0);
  static int64_t read_int64_le(const std::vector<uint8_t>& vec, size_type offset = 0);

  const std::vector<uint8_t>& GetStdVector() const { return *v;}

private:
  std::vector<uint8_t>* v;
};
} // namespace Anki

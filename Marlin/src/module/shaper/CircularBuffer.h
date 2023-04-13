#pragma once

#include <cstdint>
#include "../../MarlinCore.h"
#include "../../../../snapmaker/debug/debug.h"

// template<typename T> // T is circular_buffer type
// class circular_buffer_iterator {
//   public:
//     typedef T cbuf_type;
//     circular_buffer_iterator(
//          cbuf_type *b, size_t start_pos)
//          : buf_(b), pos_(start_pos) {}
//     T &operator*() { return (*buf_)[pos_]; }
//     T *operator->() { return &(operator*()); }
//     T &operator++() { pos_++; return *this; }

//   private:
//     cbuf_type *buf_;
//     size_t     pos_;
// };

template <class T>
class circular_buffer {

  typedef T          value_type;
  typedef T         *pointer;
  typedef const T   *const_pointer;
  typedef T         &reference;
  typedef const T   &const_reference;
  typedef size_t     size_type;
  typedef ptrdiff_t  difference_type;
  // typedef circular_buffer_iterator<circular_buffer> iterator;
  // iterator begin() { return iterator(this, 0); }
  // iterator end() { return iterator(this, size()); }

  public:
    explicit circular_buffer(size_t capacity = 64)
        : array_(new T[capacity]),
          array_size_(capacity),
          head_(0), tail_(0) {}
    ~circular_buffer() { delete [] array_; }
    size_t capacity() const { return array_size_; }
    size_t size() const { return (head_ - tail_) % array_size_; }

    reference front() { return array_[head_]; }
    reference back() { return array_[tail_]; }
    const_reference front() const { return array_[head_]; }
    const_reference back() const { return array_[tail_]; }
    void clear() { head_ = tail_ = 0; }

    size_type next(size_type idx) const { return (idx + 1) % array_size_; }
    bool is_empty() const { return head_ == tail_; }
    bool is_full() const { return next(head_) == tail_; }
    void increment_tail() { tail_ = next(tail_); }
    void increment_head(){ head_ = next(head_); }

    bool push(const value_type &item) {
      if (!is_full()) {
        array_[head_] = item;
        increment_head();
        return true;
      }
      else {
        return false;
      }
    }
    bool pop(value_type &item) {
      if (!is_empty()) {
        item = array_[tail_];
        increment_tail();
        return true;
      }
      else {
        return false;
      }
    }

    value_type &operator[](size_type idx) { return array_[idx]; }
    const value_type &operator[](size_type idx) const { return array_[idx]; }

  private:
    T      *array_;
    size_t  array_size_;
    size_t  head_;
    size_t  tail_;
};
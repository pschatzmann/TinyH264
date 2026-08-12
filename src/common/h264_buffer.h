#pragma once
#include <stddef.h>
#include <new>

/*
 * Header-only. Buffer<T, Allocator>: this project's std::vector
 * replacement everywhere a heap-allocated, allocate-once array is
 * needed - Frame<Allocator>'s Y/U/V planes (common/h264_frame.h) and
 * MbInfoTable's per-macroblock metadata (common/h264_mb_info.h) both use
 * it, so it lives here rather than inside either one specifically.
 *
 * Deliberately not a general-purpose vector: std::vector's contract
 * assumes Allocator::allocate() either succeeds or throws, and *writes
 * through* the pointer it returns (to value-initialize the new elements)
 * before a caller ever gets to inspect it. With a no-throw Allocator (see
 * ../StdAllocator.h's StdAllocator, used as this project's default) that
 * returns nullptr on failure instead of throwing, that write-before-check
 * ordering is a null-pointer write - this was verified empirically on
 * this project's own toolchain (g++/libstdc++): it segfaults at -O0, and
 * at -O2 the optimizer proves the null write is undefined behavior and
 * elides it entirely, leaving the vector in an inconsistent state
 * (size() > 0 but data() == nullptr) that crashes or corrupts memory
 * later instead, at some less obvious point. Buffer sidesteps this by
 * calling Allocator::allocate() itself and checking the returned pointer
 * *before* anything is written through it - never delegating that check
 * to a container that assumes it can't fail.
 *
 * Deliberately narrow in the other direction too: every user of this
 * class only ever allocates its storage once (allocate()) and either
 * keeps it or releases it outright (release()) - there's no mid-life
 * resize/grow the way std::vector supports, so this doesn't implement
 * one.
 */

namespace tinyh264 {

/**
 * Fixed-capacity, allocate-once array of T, backed by `Allocator`
 * (expected to be a no-throw allocator - see the file comment above and
 * ../StdAllocator.h). Elements are placement-new default-constructed on
 * allocate() and destroyed on release(), same as std::vector would -
 * this matters for correctness (not just as a formality) for non-trivial
 * element types, and is free (optimized away entirely) for the trivial
 * ones (uint8_t, int16_t, aggregate structs of only trivial members)
 * this project actually instantiates it with.
 */
template <typename T, typename Allocator>
class Buffer {
 public:
  Buffer() = default;
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;
  Buffer(Buffer&& other) noexcept { moveFrom(other); }
  Buffer& operator=(Buffer&& other) noexcept {
    if (this != &other) {
      release();
      moveFrom(other);
    }
    return *this;
  }
  ~Buffer() { release(); }

  bool empty() const { return data_ == nullptr; }
  size_t size() const { return size_; }
  T* data() { return data_; }
  const T* data() const { return data_; }
  T& operator[](size_t i) { return data_[i]; }
  const T& operator[](size_t i) const { return data_[i]; }

  /**
   * Allocates storage for `n` elements if this Buffer doesn't already
   * hold any (a no-op, returning true, otherwise - matching the old
   * std::vector-based ensureAllocated()/reset()'s "allocate once"
   * contract). Returns false, without throwing and without writing
   * through a possibly-null pointer, if the allocator reports failure -
   * either a null return from a no-throw Allocator (the expected case,
   * see ../StdAllocator.h), or - for a throwing Allocator like plain
   * std::allocator<T> - a caught std::bad_alloc, when this is built with
   * C++ exceptions enabled.
   */
  bool allocate(size_t n) {
    if (data_) return true;
    T* p = tryAllocate(n);
    if (!p) return false;
    for (size_t i = 0; i < n; i++) new (&p[i]) T();
    data_ = p;
    size_ = n;
    return true;
  }

  /// Destroys every element and frees this Buffer's storage (a no-op if not currently allocated).
  void release() {
    if (data_) {
      for (size_t i = 0; i < size_; i++) data_[i].~T();
      alloc_.deallocate(data_, size_);
      data_ = nullptr;
      size_ = 0;
    }
  }

 private:
  T* tryAllocate(size_t n) {
#if __cpp_exceptions
    try {
      return alloc_.allocate(n);
    } catch (...) {
      return nullptr;
    }
#else
    return alloc_.allocate(n);
#endif
  }

  void moveFrom(Buffer& other) {
    data_ = other.data_;
    size_ = other.size_;
    alloc_ = other.alloc_;
    other.data_ = nullptr;
    other.size_ = 0;
  }

  T* data_ = nullptr;
  size_t size_ = 0;
  Allocator alloc_;
};

}  // namespace tinyh264

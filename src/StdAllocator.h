#pragma once
#include <cstddef>
#include <cstdlib>

/*
 * Header-only. StdAllocator<T>: TinyH264's own default allocator - a
 * minimal C++ Allocator (std::vector/std::allocator-compatible) that
 * behaves like std::allocator<T> except for one deliberate difference:
 * allocate() never throws. It returns nullptr on failure instead, the
 * contract common/h264_frame.h's Buffer<Allocator> relies on to turn an
 * out-of-memory condition into DecodeStatus::kAllocationError /
 * TinyH264Decoder::Status::kAllocationError (and the encoder's analogous
 * "return 0" convention) instead of a crash.
 *
 * Why not just keep std::allocator<uint8_t> (the STL default, and what
 * this project used before)? std::allocator::allocate() throws
 * std::bad_alloc on failure - fine on a desktop, but wrong on a
 * microcontroller for two reasons: (1) many embedded C++ toolchains
 * (including common ESP-IDF/Arduino-ESP32 configurations) are built with
 * C++ exceptions disabled entirely, in which case a thrown std::bad_alloc
 * calls std::terminate()/abort() immediately, regardless of any try/catch
 * further up the call stack; (2) even with exceptions enabled, nothing in
 * this codebase used to catch that exception, so it propagated all the
 * way out of Decoder<Allocator>/Encoder<Allocator>/TinyH264Decoder<Allocator>/
 * TinyH264Encoder<Allocator> uncaught - still a crash (std::terminate),
 * just via a different mechanism. Making the default allocator itself
 * never throw means Buffer<Allocator>'s plain null-pointer check (see
 * h264_frame.h) is sufficient to catch an allocation failure on every
 * toolchain, exceptions enabled or not - and a caller sees
 * DecodeStatus::kAllocationError / Status::kAllocationError / a 0 return
 * from encodeFrame(), instead of the process going down.
 *
 * This was verified empirically, not just reasoned about: plugging a
 * null-returning allocator into a plain std::vector<uint8_t, Allocator>
 * segfaults inside resize() at -O0 (it writes through the pointer to
 * value-initialize the new elements before any caller code can check it),
 * and at -O2 the optimizer proves that write is undefined behavior and
 * elides it, leaving the vector in an inconsistent state (size() > 0 but
 * data() == nullptr) that corrupts something else later instead. That's
 * why Buffer<Allocator> (h264_frame.h) exists rather than continuing to
 * use std::vector directly for these buffers - see that file's comment.
 *
 * PSRAMAllocatorESP32 (see that file) follows the same no-throw contract,
 * for the same reason - both are safe to use as Buffer<Allocator>'s
 * Allocator. A caller who explicitly passes plain std::allocator<uint8_t>
 * instead (or any other Allocator whose allocate() can throw) is still
 * supported as long as C++ exceptions are enabled - Buffer<Allocator>
 * catches std::bad_alloc from it and converts it to the same
 * false/kAllocationError result - but on an exceptions-disabled build,
 * that specific combination is the caller's own risk (nothing can safely
 * intercept a throw when the toolchain can't unwind).
 */

namespace tinyh264 {

/**
 * TinyH264's default allocator: std::allocator-shaped (usable as
 * std::vector's or Buffer's second template argument), backed by the
 * regular heap (malloc()/free()), but never throws - see this file's
 * comment. Stateless (all instances compare equal), as required for a
 * standard-library Allocator.
 */
template <typename T>
class StdAllocator {
 public:
  using value_type = T;

  /// Default-constructs a stateless allocator instance.
  StdAllocator() noexcept = default;

  /**
   * Rebinding constructor required by the Allocator concept (lets
   * std::vector<T, StdAllocator<T>>/Buffer<T, StdAllocator<T>> construct
   * the allocator for an internal element type U from one for T).
   */
  template <typename U>
  StdAllocator(const StdAllocator<U>&) noexcept {}

  /**
   * Allocates storage for `n` objects of type T from the regular heap.
   * Returns nullptr - never throws - if `n` would overflow the byte-size
   * multiplication, or if malloc() itself fails.
   */
  T* allocate(std::size_t n) noexcept {
    if (n > (std::size_t(-1) / sizeof(T))) return nullptr;
    return static_cast<T*>(malloc(n * sizeof(T)));
  }

  /// Frees storage previously returned by allocate().
  void deallocate(T* p, std::size_t) noexcept { free(p); }

  /**
   * Allocators of this type are stateless, so any two instances (even for
   * different T) are interchangeable.
   */
  template <typename U>
  bool operator==(const StdAllocator<U>&) const noexcept {
    return true;
  }
  /// See operator==; always false since operator== is always true.
  template <typename U>
  bool operator!=(const StdAllocator<U>&) const noexcept {
    return false;
  }
};

}  // namespace tinyh264

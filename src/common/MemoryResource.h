#pragma once
#include <cstddef>
#include <cstdint>

/*
 * Header-only. A small, byte-oriented, type-erased allocator interface
 * (the std::pmr::memory_resource pattern) - lets the low-level software
 * codec classes (SoftwareEncoder/SoftwareDecoder, and the Frame/Buffer/
 * MbInfoTable types they share) take an allocation strategy as a
 * *runtime* constructor argument instead of a compile-time template
 * parameter, while TinyH264Encoder<Allocator>/TinyH264Decoder<Allocator>
 * keep their existing templated public API unchanged - they build one
 * of these (via AllocatorMemoryResource below) from their own Allocator
 * template argument and hand it down.
 *
 * Allocation happens once per Frame/Buffer/MbInfoTable, at open()/
 * setMaxDimension() time - never in the per-frame encode/decode hot
 * path - so the one virtual call per allocate()/deallocate() this adds
 * is negligible; nothing here is called per frame.
 */

namespace tinyh264 {

/**
 * Type-erased allocator interface. allocate() is deliberately not
 * noexcept: StdAllocator/PSRAMAllocatorESP32 (this project's own
 * allocators, see StdAllocator.h/PSRAMAllocatorESP32.h) never throw and
 * return nullptr on failure instead, but a caller can also plug in an
 * ordinary std::allocator<uint8_t> (test/native/test_lifecycle.cpp
 * does), which does throw on failure - Buffer<T>::tryAllocate()
 * (h264_buffer.h) already wraps its allocation call in a try/catch when
 * exceptions are enabled, converting either behavior into the same
 * null-return contract, so this interface just needs to let an
 * exception propagate through it rather than promising not to.
 */
class MemoryResource {
 public:
  virtual ~MemoryResource() = default;
  virtual uint8_t* allocate(size_t bytes) = 0;
  virtual void deallocate(uint8_t* p, size_t bytes) noexcept = 0;
};

/**
 * Adapts any existing std::allocator-shaped type (StdAllocator<uint8_t>,
 * PSRAMAllocatorESP32<uint8_t>, std::allocator<uint8_t>, ...) into a
 * MemoryResource. Stateless-by-construction, matching every current
 * Allocator's own contract (see StdAllocator.h's file comment) - one
 * instance is created once, inside TinyH264Encoder<Allocator>/
 * TinyH264Decoder<Allocator>, from their own Allocator template
 * argument.
 */
template <typename Allocator>
class AllocatorMemoryResource : public MemoryResource {
 public:
  uint8_t* allocate(size_t bytes) override { return alloc_.allocate(bytes); }
  void deallocate(uint8_t* p, size_t bytes) noexcept override {
    alloc_.deallocate(p, bytes);
  }

 private:
  Allocator alloc_;
};

}  // namespace tinyh264

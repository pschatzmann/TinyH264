#pragma once
#include <stdarg.h>
#include <stdio.h>

#ifdef ARDUINO
#include <Print.h>

/*
 * Header-only. Logger: a small, leveled, printf-style logger for Arduino
 * sketches - not currently used internally by TinyH264 itself (which
 * only ever writes a couple of fixed diagnostic lines straight to
 * Serial, see TinyH264Encoder.h's hardware-encoder fallback messages),
 * but useful for a sketch that wants leveled, silenceable logging
 * without pulling in a separate dependency.
 *
 * Usage:
 *   using namespace tinyh264;
 *   Logger logger;
 *   logger.begin(LogLevel::kInfo);      // output defaults to Serial
 *   logger.info("starting encode, %dx%d", width, height);
 *   logger.warn("dropped %d frame(s)", dropped);
 *   logger.error("open() failed: %d", err);
 *   logger.debug("mv=(%d,%d)", mv[0], mv[1]);  // suppressed at kInfo
 *
 * Each level includes every level above it in severity (kError <
 * kWarn < kInfo < kDebug numerically; a call is printed when its own
 * level is <= the configured level) - the usual "-v/-vv/-vvv" model.
 * kNone disables logging entirely, regardless of which method is
 * called - useful as a single off-switch without touching call sites.
 *
 * begin()'s `output` parameter takes any Print& (Serial, a second
 * UART, a file, a network client, ...), not just Serial - Serial is
 * only the default so `logger.begin();` with no arguments works
 * out of the box. A default-constructed, never-begin()'d Logger is
 * silent (no output, see out_'s default below) rather than writing to
 * Serial before the sketch has necessarily called Serial.begin() - so
 * begin() is required, not optional, before anything prints.
 */

namespace tinyh264 {

/// Log severity, most to least severe. Values only ever compared to each
/// other via <= (see Logger::log()), never serialized, so the exact
/// numbering isn't a compatibility concern.
enum class LogLevel : uint8_t {
  kNone = 0,   ///< logging disabled entirely
  kError = 1,  ///< error() only
  kWarn = 2,   ///< + warn()
  kInfo = 3,   ///< + info()
  kDebug = 4,  ///< + debug() - everything
};

class Logger {
 public:
  /**
   * Configures this Logger's active level and output destination.
   * Must be called before anything logs (see the file comment above for
   * why there's no implicit default output) - typically once, from
   * setup(). Safe to call again later to change the level and/or
   * redirect output.
   */
  void begin(LogLevel level = LogLevel::kInfo, Print& output = Serial) {
    level_ = level;
    out_ = &output;
  }

  /// Changes the active level without touching the output destination.
  void setLevel(LogLevel level) { level_ = level; }
  LogLevel level() const { return level_; }

  void error(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    log(LogLevel::kError, "ERROR", fmt, args);
    va_end(args);
  }

  void warn(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    log(LogLevel::kWarn, "WARN", fmt, args);
    va_end(args);
  }

  void info(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    log(LogLevel::kInfo, "INFO", fmt, args);
    va_end(args);
  }

  void debug(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list args;
    va_start(args, fmt);
    log(LogLevel::kDebug, "DEBUG", fmt, args);
    va_end(args);
  }

 private:
  // Fixed-size stack buffer rather than a heap allocation - logging
  // should never be the thing that fails an allocation-constrained
  // sketch. A message longer than this is truncated by vsnprintf (never
  // overflowed) rather than dropped outright.
  static constexpr size_t kBufSize = 160;

  void log(LogLevel msgLevel, const char* tag, const char* fmt,
           va_list args) {
    if (out_ == nullptr || level_ == LogLevel::kNone ||
        (uint8_t)msgLevel > (uint8_t)level_) {
      return;
    }
    char buf[kBufSize];
    vsnprintf(buf, sizeof(buf), fmt, args);
    out_->print(tag);
    out_->print(": ");
    out_->println(buf);
  }

  LogLevel level_ = LogLevel::kNone;
  Print* out_ = nullptr;
};

}  // namespace tinyh264

#endif  // ARDUINO

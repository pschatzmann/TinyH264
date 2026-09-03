#pragma once
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#ifdef ARDUINO
#include <Print.h>
#endif

/*
 * Header-only. Logger: a small, leveled, printf-style logger - not
 * currently used internally by TinyH264 itself (which only ever writes
 * a couple of fixed diagnostic lines straight to Serial, see
 * TinyH264Encoder.h's hardware-encoder fallback messages), but useful
 * for a caller that wants leveled, silenceable logging without pulling
 * in a separate dependency. Available in both an Arduino sketch build
 * and a plain host/desktop build (e.g. this project's own test/native
 * suite) - only the *destination* differs: on Arduino,
 * begin() takes any Print& (Serial, a second UART, a file, a network
 * client, ...), defaulting to Serial; off Arduino, there's no Print
 * concept at all, so every message goes straight to printf() (stdout)
 * instead - same log()/info()/warn()/error()/debug() call sites either
 * way, just a different `begin()` signature (see its own comment).
 *
 * Usage:
 *   using namespace tinyh264;
 *   Logger logger;
 *   logger.begin(LogLevel::kInfo);      // Arduino: output defaults to Serial
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
 * A default-constructed, never-begin()'d Logger is silent rather than
 * writing anything (see level_'s default below) - on Arduino, that also
 * avoids writing to Serial before the sketch has necessarily called
 * Serial.begin() - so begin() is required, not optional, before
 * anything prints, on both platforms.
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
#ifdef ARDUINO
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
#else
  /**
   * Configures this Logger's active level - off this platform, output
   * is always printf() (stdout), so there's no destination to pass in.
   * Must be called before anything logs (see the file comment above);
   * safe to call again later to change the level.
   */
  void begin(LogLevel level = LogLevel::kInfo) { level_ = level; }
#endif

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
#ifdef ARDUINO
  // Fixed-size stack buffer rather than a heap allocation - logging
  // should never be the thing that fails an allocation-constrained
  // sketch. A message longer than this is truncated by vsnprintf (never
  // overflowed) rather than dropped outright. Not needed off Arduino -
  // vprintf() there writes straight through, no intermediate buffer.
  static constexpr size_t kBufSize = 160;
#endif

  void log(LogLevel msgLevel, const char* tag, const char* fmt,
           va_list args) {
    if (level_ == LogLevel::kNone || (uint8_t)msgLevel > (uint8_t)level_) {
      return;
    }
#ifdef ARDUINO
    if (out_ == nullptr) return;
    char buf[kBufSize];
    vsnprintf(buf, sizeof(buf), fmt, args);
    out_->print(tag);
    out_->print(": ");
    out_->println(buf);
#else
    printf("%s: ", tag);
    vprintf(fmt, args);
    printf("\n");
#endif
  }

  LogLevel level_ = LogLevel::kNone;
#ifdef ARDUINO
  Print* out_ = nullptr;
#endif
};

/// Shared logger instance for the whole library - silent (kNone) until
/// a caller opts in, e.g. `tinyh264::H264LOG.begin(LogLevel::kInfo);`
/// once from setup(). A C++17 inline variable: exactly one instance
/// across however many translation units end up including this header
/// (unlike a plain `static`, which would give each translation unit
/// its own copy) - the correct header-only-library way to share state
/// like this without a getter function or a definition living in some
/// separate .cpp file this project doesn't otherwise have.
inline Logger H264LOG;

}  // namespace tinyh264

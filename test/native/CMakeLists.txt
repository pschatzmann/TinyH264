# Desktop-only test suite: each test_*.cpp is a standalone executable
# that decodes real ffmpeg/libx264-generated streams (assets/*.264) and
# diffs the result against ffmpeg's own decode (assets/*.yuv, *_ref.raw)
# - this project's correctness oracle, not conformance-suite guesswork.
# See the comment at the top of each test file for exactly what it
# validates. None of this runs on the embedded target; it's a host-only
# (native g++/clang/MSVC) build against the same headers examples/ uses.
#
# Explicit list rather than a glob: CMake doesn't notice new files added
# to a glob without a manual re-configure, which silently produces a
# stale test suite - not worth the convenience for a list this short-lived.
set(TINYH264_TESTS
    test_alloc_failure
    test_bitreader
    test_cavlc_tables
    test_decode_debug
    test_decode_flat
    test_decode_flat_multi
    test_decode_iframe
    test_decode_multiframe
    test_decode_multiframe_nodbf
    test_decode_multiref
    test_encode_autoframe
    test_encode_color_formats
    test_encode_i4x4
    test_encode_iframe
    test_encode_pframe
    test_encode_pslice_intra_fallback
    test_encode_ratecontrol
    test_encode_setters
    test_lifecycle
    test_logger_wiring
    test_max_dimension
    test_motion_search_fast
    test_motion_search_range
    test_nal
    test_public_api
    test_rgb565
    test_rgb666
    test_rgb888
    test_scale
    test_slice_header
    test_sps_pps
    test_yuv420
)

foreach(test_name ${TINYH264_TESTS})
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE TinyH264)
    add_test(NAME ${test_name} COMMAND ${test_name})
    # Every test reads its assets via relative paths (e.g. "assets/multiref.264"),
    # so it must run with test/native/ as the working directory regardless
    # of where the build tree actually lives.
    set_tests_properties(${test_name} PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

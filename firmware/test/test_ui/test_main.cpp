#include <unity.h>

#include <cstring>

#include "core/Version.h"
#include "core/ui/VersionBadge.h"

using qpcr::ui::kVersionBadgeMargin;
using qpcr::ui::Point;
using qpcr::ui::TextMetrics;
using qpcr::ui::versionBadgeLabel;
using qpcr::ui::versionBadgeOrigin;

void setUp() {}
void tearDown() {}

void test_label_prefixes_v() {
    char out[32];
    const std::size_t n = versionBadgeLabel(out, sizeof out, "1.2.3");
    TEST_ASSERT_EQUAL_STRING("v1.2.3", out);
    TEST_ASSERT_EQUAL_UINT(6, n);
}

void test_label_keeps_existing_v() {
    char out[32];
    versionBadgeLabel(out, sizeof out, "v1.2.3");
    TEST_ASSERT_EQUAL_STRING("v1.2.3", out);
}

void test_label_shows_fallback_verbatim() {
    char out[32];
    versionBadgeLabel(out, sizeof out, "0.0.0-unversioned");
    TEST_ASSERT_EQUAL_STRING("v0.0.0-unversioned", out);
}

void test_label_marks_missing_version() {
    char out[32];
    versionBadgeLabel(out, sizeof out, nullptr);
    TEST_ASSERT_EQUAL_STRING("v?", out);
    versionBadgeLabel(out, sizeof out, "");
    TEST_ASSERT_EQUAL_STRING("v?", out);
}

void test_label_truncates_and_terminates() {
    char out[5];
    std::memset(out, 'X', sizeof out);
    const std::size_t n = versionBadgeLabel(out, sizeof out, "12.34.56");
    TEST_ASSERT_EQUAL_STRING("v12.", out);
    TEST_ASSERT_EQUAL_UINT(4, n);
    TEST_ASSERT_EQUAL_UINT(0, versionBadgeLabel(out, 0, "1.0.0"));
    TEST_ASSERT_EQUAL_UINT(0, versionBadgeLabel(nullptr, sizeof out, "1.0.0"));
}

void test_label_uses_firmware_version() {
    char out[48];
    versionBadgeLabel(out, sizeof out, qpcr::firmwareVersion());
    TEST_ASSERT_EQUAL_CHAR('v', out[0]);
    TEST_ASSERT_EQUAL_STRING(qpcr::firmwareVersion(), out + 1);
}

void test_origin_hugs_bottom_right_with_margin() {
    const Point p = versionBadgeOrigin(480, 320, TextMetrics{60, 8});
    TEST_ASSERT_EQUAL_INT(480 - kVersionBadgeMargin - 60, p.x);
    TEST_ASSERT_EQUAL_INT(320 - kVersionBadgeMargin - 8, p.y);
}

void test_origin_follows_rotation() {
    const Point landscape = versionBadgeOrigin(480, 320, TextMetrics{60, 8});
    const Point portrait = versionBadgeOrigin(320, 480, TextMetrics{60, 8});
    TEST_ASSERT_EQUAL_INT(416, landscape.x);
    TEST_ASSERT_EQUAL_INT(308, landscape.y);
    TEST_ASSERT_EQUAL_INT(256, portrait.x);
    TEST_ASSERT_EQUAL_INT(468, portrait.y);
}

void test_origin_never_negative() {
    const Point p = versionBadgeOrigin(40, 10, TextMetrics{200, 16});
    TEST_ASSERT_EQUAL_INT(0, p.x);
    TEST_ASSERT_EQUAL_INT(0, p.y);
}

void test_origin_custom_margin() {
    const Point p = versionBadgeOrigin(100, 100, TextMetrics{10, 10}, 0);
    TEST_ASSERT_EQUAL_INT(90, p.x);
    TEST_ASSERT_EQUAL_INT(90, p.y);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_label_prefixes_v);
    RUN_TEST(test_label_keeps_existing_v);
    RUN_TEST(test_label_shows_fallback_verbatim);
    RUN_TEST(test_label_marks_missing_version);
    RUN_TEST(test_label_truncates_and_terminates);
    RUN_TEST(test_label_uses_firmware_version);
    RUN_TEST(test_origin_hugs_bottom_right_with_margin);
    RUN_TEST(test_origin_follows_rotation);
    RUN_TEST(test_origin_never_negative);
    RUN_TEST(test_origin_custom_margin);
    return UNITY_END();
}

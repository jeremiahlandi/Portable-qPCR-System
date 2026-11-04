#include <unity.h>

#include <cstring>

#include "core/Version.h"
#include "core/ui/Launcher.h"
#include "core/ui/StatusBar.h"
#include "core/ui/VersionBadge.h"

using qpcr::ui::BatteryLevel;
using qpcr::ui::batteryLevel;
using qpcr::ui::formatClock;
using qpcr::ui::kLauncherTiles;
using qpcr::ui::kVersionBadgeMargin;
using qpcr::ui::launcherTileIndex;
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

void test_battery_level_thresholds() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Unknown),
                          static_cast<int>(batteryLevel(-1)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Empty), static_cast<int>(batteryLevel(0)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Empty), static_cast<int>(batteryLevel(9)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Low), static_cast<int>(batteryLevel(10)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Half), static_cast<int>(batteryLevel(50)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::High), static_cast<int>(batteryLevel(89)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Full), static_cast<int>(batteryLevel(90)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(BatteryLevel::Full),
                          static_cast<int>(batteryLevel(250)));
}

void test_clock_formats_and_wraps() {
    char out[8];
    TEST_ASSERT_EQUAL_UINT(5, formatClock(out, sizeof out, 0));
    TEST_ASSERT_EQUAL_STRING("00:00", out);
    formatClock(out, sizeof out, 61);
    TEST_ASSERT_EQUAL_STRING("00:01", out);
    formatClock(out, sizeof out, 13 * 3600 + 7 * 60 + 59);
    TEST_ASSERT_EQUAL_STRING("13:07", out);
    formatClock(out, sizeof out, 24 * 3600 + 5 * 60);
    TEST_ASSERT_EQUAL_STRING("00:05", out);
}

void test_clock_rejects_small_buffer() {
    char out[5];
    TEST_ASSERT_EQUAL_UINT(0, formatClock(out, sizeof out, 3600));
    TEST_ASSERT_EQUAL_STRING("", out);
    TEST_ASSERT_EQUAL_UINT(0, formatClock(nullptr, 8, 3600));
}

void test_launcher_tiles_have_unique_ids() {
    for (std::size_t i = 0; i < kLauncherTiles.size(); ++i) {
        TEST_ASSERT_NOT_NULL(kLauncherTiles.at(i).id);
        TEST_ASSERT_TRUE(std::strlen(kLauncherTiles.at(i).label) > 0);
        TEST_ASSERT_EQUAL_UINT(i, launcherTileIndex(kLauncherTiles.at(i).id));
    }
    TEST_ASSERT_EQUAL_UINT(kLauncherTiles.size(), launcherTileIndex("nope"));
    TEST_ASSERT_EQUAL_UINT(kLauncherTiles.size(), launcherTileIndex(nullptr));
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
    RUN_TEST(test_battery_level_thresholds);
    RUN_TEST(test_clock_formats_and_wraps);
    RUN_TEST(test_clock_rejects_small_buffer);
    RUN_TEST(test_launcher_tiles_have_unique_ids);
    return UNITY_END();
}

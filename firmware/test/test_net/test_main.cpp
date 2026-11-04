#include <unity.h>

#include <cstring>

#include "core/net/WifiSettings.h"
#include "core/time/TimeZones.h"

using qpcr::net::ConnectEvent;
using qpcr::net::ConnectState;
using qpcr::net::isFailure;
using qpcr::net::nextState;
using qpcr::net::stateText;
using qpcr::net::validPassphrase;
using qpcr::net::validSsid;
using qpcr::time::kDefaultTimeZone;
using qpcr::time::kTimeZones;
using qpcr::time::secondsOfDay;
using qpcr::time::timeZoneIndex;

void setUp() {}
void tearDown() {}

void test_ssid_rules() {
    TEST_ASSERT_TRUE(validSsid("FarmOffice"));
    TEST_ASSERT_TRUE(validSsid("a"));
    TEST_ASSERT_TRUE(validSsid("12345678901234567890123456789012"));   // 32 bytes
    TEST_ASSERT_FALSE(validSsid("123456789012345678901234567890123")); // 33 bytes
    TEST_ASSERT_FALSE(validSsid(""));
    TEST_ASSERT_FALSE(validSsid(nullptr));
    TEST_ASSERT_FALSE(validSsid("bad\tname"));
}

void test_passphrase_rules() {
    TEST_ASSERT_TRUE(validPassphrase(""));         // open network
    TEST_ASSERT_TRUE(validPassphrase("12345678")); // 8
    TEST_ASSERT_FALSE(validPassphrase("1234567")); // 7
    char sixtyThree[64];
    std::memset(sixtyThree, 'x', 63);
    sixtyThree[63] = '\0';
    TEST_ASSERT_TRUE(validPassphrase(sixtyThree));
    char sixtyFour[65];
    std::memset(sixtyFour, 'x', 64);
    sixtyFour[64] = '\0';
    TEST_ASSERT_FALSE(validPassphrase(sixtyFour));
    TEST_ASSERT_FALSE(validPassphrase(nullptr));
    TEST_ASSERT_FALSE(validPassphrase("has\x01control"));
}

void test_connect_flow_happy_path_secured() {
    ConnectState s = ConnectState::Idle;
    s = nextState(s, ConnectEvent::ScanStarted);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectState::Scanning), static_cast<int>(s));
    s = nextState(s, ConnectEvent::ScanDone);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectState::Selecting), static_cast<int>(s));
    s = nextState(s, ConnectEvent::ChoseSecuredNetwork);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectState::EnteringPassword), static_cast<int>(s));
    s = nextState(s, ConnectEvent::PasswordEntered);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectState::Connecting), static_cast<int>(s));
    s = nextState(s, ConnectEvent::LinkUp);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectState::Connected), static_cast<int>(s));
    TEST_ASSERT_EQUAL_STRING("Connected", stateText(s));
}

void test_connect_flow_open_network_skips_password() {
    ConnectState s = ConnectState::Selecting;
    s = nextState(s, ConnectEvent::ChoseOpenNetwork);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ConnectState::Connecting), static_cast<int>(s));
}

void test_connect_flow_failures_and_recovery() {
    TEST_ASSERT_TRUE(isFailure(nextState(ConnectState::Connecting, ConnectEvent::WrongPassword)));
    TEST_ASSERT_TRUE(isFailure(nextState(ConnectState::Connecting, ConnectEvent::NotFound)));
    TEST_ASSERT_TRUE(isFailure(nextState(ConnectState::Connecting, ConnectEvent::Timeout)));
    TEST_ASSERT_FALSE(isFailure(ConnectState::Connected));
    // A failure outside Connecting is ignored.
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ConnectState::Selecting),
        static_cast<int>(nextState(ConnectState::Selecting, ConnectEvent::WrongPassword)));
    // Cancel returns to Idle from anywhere but Connected; Forget always does.
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ConnectState::Idle),
        static_cast<int>(nextState(ConnectState::EnteringPassword, ConnectEvent::Cancel)));
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ConnectState::Connected),
        static_cast<int>(nextState(ConnectState::Connected, ConnectEvent::Cancel)));
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ConnectState::Idle),
        static_cast<int>(nextState(ConnectState::Connected, ConnectEvent::Forget)));
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(ConnectState::Idle),
        static_cast<int>(nextState(ConnectState::Connected, ConnectEvent::LinkLost)));
    TEST_ASSERT_EQUAL_STRING("Wrong password", stateText(ConnectState::FailedWrongPassword));
}

void test_time_zone_lookup() {
    TEST_ASSERT_EQUAL_UINT(1, timeZoneIndex("us-eastern"));
    TEST_ASSERT_EQUAL_UINT(0, timeZoneIndex("utc"));
    TEST_ASSERT_EQUAL_UINT(kDefaultTimeZone, timeZoneIndex("mars"));
    TEST_ASSERT_EQUAL_UINT(kDefaultTimeZone, timeZoneIndex(nullptr));
    for (const auto& z : kTimeZones) {
        TEST_ASSERT_TRUE(std::strlen(z.id) > 0);
        TEST_ASSERT_TRUE(std::strlen(z.label) > 0);
        TEST_ASSERT_TRUE(std::strlen(z.posix) > 0);
    }
}

void test_seconds_of_day() {
    TEST_ASSERT_EQUAL_UINT(0, secondsOfDay(0, 0, 0));
    TEST_ASSERT_EQUAL_UINT(13 * 3600 + 7 * 60 + 59, secondsOfDay(13, 7, 59));
    TEST_ASSERT_EQUAL_UINT(5 * 60, secondsOfDay(24, 5, 0));
    TEST_ASSERT_EQUAL_UINT(23 * 3600, secondsOfDay(-1, 0, 0));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_ssid_rules);
    RUN_TEST(test_passphrase_rules);
    RUN_TEST(test_connect_flow_happy_path_secured);
    RUN_TEST(test_connect_flow_open_network_skips_password);
    RUN_TEST(test_connect_flow_failures_and_recovery);
    RUN_TEST(test_time_zone_lookup);
    RUN_TEST(test_seconds_of_day);
    return UNITY_END();
}

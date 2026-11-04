#include <unity.h>

#include <cctype>
#include <cstring>

#include "core/Version.h"

void setUp() {}
void tearDown() {}

// version.txt holds a semantic version (Release Please). The embedded string must be
// non-empty and start with "major.minor.patch".
void test_version_is_semver() {
    const char* v = qpcr::firmwareVersion();
    TEST_ASSERT_NOT_NULL(v);
    TEST_ASSERT_TRUE(std::strlen(v) >= 5);
    int dots = 0;
    for (const char* p = v; *p != '\0' && *p != '-' && *p != '+'; ++p) {
        if (*p == '.') {
            ++dots;
        } else {
            TEST_ASSERT_TRUE_MESSAGE(std::isdigit(static_cast<unsigned char>(*p)) != 0, v);
        }
    }
    TEST_ASSERT_EQUAL_MESSAGE(2, dots, v);
}

// The build string is the release number, optionally followed by "+<commits>.g<sha>[.dirty]".
void test_build_extends_version() {
    const char* v = qpcr::firmwareVersion();
    const char* b = qpcr::firmwareBuild();
    TEST_ASSERT_NOT_NULL(b);
    TEST_ASSERT_EQUAL_STRING_LEN(v, b, std::strlen(v));
    const char* rest = b + std::strlen(v);
    if (*rest != '\0') {
        TEST_ASSERT_EQUAL_CHAR('+', *rest);
        TEST_ASSERT_NOT_NULL(std::strstr(rest, ".g"));
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_version_is_semver);
    RUN_TEST(test_build_extends_version);
    return UNITY_END();
}

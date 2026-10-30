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

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_version_is_semver);
    return UNITY_END();
}

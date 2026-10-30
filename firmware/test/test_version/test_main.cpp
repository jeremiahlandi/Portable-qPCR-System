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

void test_qa_sanitizer() {
    int* a = new int[4];
    a[4] = 1; // deliberate heap overflow
    delete[] a;
    TEST_ASSERT_TRUE(true);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_qa_sanitizer);
    RUN_TEST(test_version_is_semver);
    return UNITY_END();
}

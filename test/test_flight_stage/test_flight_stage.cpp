#include <unity.h>
#include "../../lib/NativeTestMocks/NativeTestHelper.h"
#include <string.h>

// include other headers you need to test here
#include "../../src/FlightStage.h"

using namespace astra_rocket;

// ---

// Set up and global variables or mocks for testing here

// ---

// These two functions are called before and after each test function, and are required in unity, even if empty.
void setUp(void)
{
    // set stuff up before each test here, if needed
}

void tearDown(void)
{
    // clean stuff up after each test here, if needed
}
// ---

// Test functions must be void and take no arguments, put them here

void test_flight_stage_enum_values() {
    // Test that all enum values are defined
    TEST_ASSERT_EQUAL(0, FlightStage::PAD_IDLE);
    TEST_ASSERT_EQUAL(1, FlightStage::BOOST);
    TEST_ASSERT_EQUAL(2, FlightStage::COAST);
    TEST_ASSERT_EQUAL(3, FlightStage::APOGEE);
    TEST_ASSERT_EQUAL(4, FlightStage::EXPECTING_DROGUE);
    TEST_ASSERT_EQUAL(5, FlightStage::UNDER_DROGUE);
    TEST_ASSERT_EQUAL(6, FlightStage::EXPECTING_MAIN);
    TEST_ASSERT_EQUAL(7, FlightStage::UNDER_MAIN);
    TEST_ASSERT_EQUAL(8, FlightStage::LANDED);
}

void test_flight_stage_to_string_pad_idle() {
    const char* str = flightStageToString(FlightStage::PAD_IDLE);
    TEST_ASSERT_EQUAL_STRING("PAD_IDLE", str);
}

void test_flight_stage_to_string_boost() {
    const char* str = flightStageToString(FlightStage::BOOST);
    TEST_ASSERT_EQUAL_STRING("BOOST", str);
}

void test_flight_stage_to_string_coast() {
    const char* str = flightStageToString(FlightStage::COAST);
    TEST_ASSERT_EQUAL_STRING("COAST", str);
}

void test_flight_stage_to_string_apogee() {
    const char* str = flightStageToString(FlightStage::APOGEE);
    TEST_ASSERT_EQUAL_STRING("APOGEE", str);
}

void test_flight_stage_to_string_expecting_drogue() {
    const char* str = flightStageToString(FlightStage::EXPECTING_DROGUE);
    TEST_ASSERT_EQUAL_STRING("EXPECTING_DROGUE", str);
}

void test_flight_stage_to_string_under_drogue() {
    const char* str = flightStageToString(FlightStage::UNDER_DROGUE);
    TEST_ASSERT_EQUAL_STRING("UNDER_DROGUE", str);
}

void test_flight_stage_to_string_expecting_main() {
    const char* str = flightStageToString(FlightStage::EXPECTING_MAIN);
    TEST_ASSERT_EQUAL_STRING("EXPECTING_MAIN", str);
}

void test_flight_stage_to_string_under_main() {
    const char* str = flightStageToString(FlightStage::UNDER_MAIN);
    TEST_ASSERT_EQUAL_STRING("UNDER_MAIN", str);
}

void test_flight_stage_to_string_landed() {
    const char* str = flightStageToString(FlightStage::LANDED);
    TEST_ASSERT_EQUAL_STRING("LANDED", str);
}

void test_flight_stage_to_string_invalid() {
    // Test with invalid enum value
    const char* str = flightStageToString(static_cast<FlightStage>(99));
    TEST_ASSERT_EQUAL_STRING("UNKNOWN", str);
}

void test_flight_stage_sequential_ordering() {
    // Verify that the enum values follow the expected flight sequence
    TEST_ASSERT_TRUE(FlightStage::PAD_IDLE < FlightStage::BOOST);
    TEST_ASSERT_TRUE(FlightStage::BOOST < FlightStage::COAST);
    TEST_ASSERT_TRUE(FlightStage::COAST < FlightStage::APOGEE);
    TEST_ASSERT_TRUE(FlightStage::APOGEE < FlightStage::EXPECTING_DROGUE);
    TEST_ASSERT_TRUE(FlightStage::EXPECTING_DROGUE < FlightStage::UNDER_DROGUE);
    TEST_ASSERT_TRUE(FlightStage::UNDER_DROGUE < FlightStage::EXPECTING_MAIN);
    TEST_ASSERT_TRUE(FlightStage::EXPECTING_MAIN < FlightStage::UNDER_MAIN);
    TEST_ASSERT_TRUE(FlightStage::UNDER_MAIN < FlightStage::LANDED);
}

void test_flight_stage_assignment() {
    // Test that enum can be assigned and compared
    FlightStage stage1 = FlightStage::PAD_IDLE;
    FlightStage stage2 = FlightStage::BOOST;

    TEST_ASSERT_EQUAL(FlightStage::PAD_IDLE, stage1);
    TEST_ASSERT_EQUAL(FlightStage::BOOST, stage2);
    TEST_ASSERT_NOT_EQUAL(stage1, stage2);
}

void test_flight_stage_switch_statement() {
    // Test that enum can be used in switch statements
    FlightStage stage = FlightStage::COAST;
    int result = 0;

    switch(stage) {
        case FlightStage::PAD_IDLE:
            result = 1;
            break;
        case FlightStage::BOOST:
            result = 2;
            break;
        case FlightStage::COAST:
            result = 3;
            break;
        case FlightStage::APOGEE:
            result = 4;
            break;
        case FlightStage::EXPECTING_DROGUE:
            result = 5;
            break;
        case FlightStage::UNDER_DROGUE:
            result = 6;
            break;
        case FlightStage::EXPECTING_MAIN:
            result = 7;
            break;
        case FlightStage::UNDER_MAIN:
            result = 8;
            break;
        case FlightStage::LANDED:
            result = 9;
            break;
    }

    TEST_ASSERT_EQUAL(3, result);
}

void test_flight_stage_to_string_all_stages() {
    // Test that all stages have valid string representations
    for (int i = 0; i <= 8; i++) {
        FlightStage stage = static_cast<FlightStage>(i);
        const char* str = flightStageToString(stage);
        TEST_ASSERT_NOT_NULL(str);
        TEST_ASSERT_TRUE(strlen(str) > 0);
        // Verify it's not "UNKNOWN" by comparing
        TEST_ASSERT_TRUE(strcmp("UNKNOWN", str) != 0);
    }
}

void test_flight_stage_count() {
    // Verify we have 9 flight stages (0-8)
    int maxStage = static_cast<int>(FlightStage::LANDED);
    TEST_ASSERT_EQUAL(8, maxStage);
}

// ---

// This is the main function that runs all the tests. It should be the last thing in the file.
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Add your tests here
    RUN_TEST(test_flight_stage_enum_values);
    RUN_TEST(test_flight_stage_to_string_pad_idle);
    RUN_TEST(test_flight_stage_to_string_boost);
    RUN_TEST(test_flight_stage_to_string_coast);
    RUN_TEST(test_flight_stage_to_string_apogee);
    RUN_TEST(test_flight_stage_to_string_expecting_drogue);
    RUN_TEST(test_flight_stage_to_string_under_drogue);
    RUN_TEST(test_flight_stage_to_string_expecting_main);
    RUN_TEST(test_flight_stage_to_string_under_main);
    RUN_TEST(test_flight_stage_to_string_landed);
    RUN_TEST(test_flight_stage_to_string_invalid);
    RUN_TEST(test_flight_stage_sequential_ordering);
    RUN_TEST(test_flight_stage_assignment);
    RUN_TEST(test_flight_stage_switch_statement);
    RUN_TEST(test_flight_stage_to_string_all_stages);
    RUN_TEST(test_flight_stage_count);

    UNITY_END();
}
// ---

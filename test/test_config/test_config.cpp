#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>

// include other headers you need to test here
#include "../../src/AstraRocketConfig.h"

using namespace astra_rocket;
using namespace astra;

// ---

// Set up and global variables or mocks for testing here
FakeBarometer fakeBaro;
FakeGPS fakeGPS;
FakeAccel fakeAccel;
FakeGyro fakeGyro;

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

void test_config_default_values() {
    AstraRocketConfig config;

    // Test default flight detection thresholds
    TEST_ASSERT_EQUAL_DOUBLE(3.0, config.getLiftoffAccelThreshold());
    TEST_ASSERT_EQUAL_UINT32(100, config.getLiftoffDetectDuration());
    TEST_ASSERT_EQUAL_DOUBLE(1.5, config.getBurnoutAccelThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(2.0, config.getApogeeVelocityThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(1.0, config.getLandingVelocityThreshold());
    TEST_ASSERT_EQUAL_UINT32(3000, config.getLandingDetectDuration());

    // Test default logging configuration
    TEST_ASSERT_EQUAL_DOUBLE(1.0, config.getPreflightLogRate());
    TEST_ASSERT_EQUAL_DOUBLE(50.0, config.getFlightLogRate());
    TEST_ASSERT_EQUAL_DOUBLE(1.0, config.getPostflightLogRate());
    TEST_ASSERT_FALSE(config.getFlashBackup());
}

void test_config_with_liftoff_threshold() {
    AstraRocketConfig config;

    // Set custom liftoff threshold
    config.withLiftoffAccelThreshold(4.5);

    TEST_ASSERT_EQUAL_DOUBLE(4.5, config.getLiftoffAccelThreshold());
}

void test_config_with_liftoff_duration() {
    AstraRocketConfig config;

    // Set custom liftoff duration
    config.withLiftoffDetectDuration(200);

    TEST_ASSERT_EQUAL_UINT32(200, config.getLiftoffDetectDuration());
}

void test_config_with_burnout_threshold() {
    AstraRocketConfig config;

    // Set custom burnout threshold
    config.withBurnoutAccelThreshold(2.0);

    TEST_ASSERT_EQUAL_DOUBLE(2.0, config.getBurnoutAccelThreshold());
}

void test_config_with_apogee_velocity_threshold() {
    AstraRocketConfig config;

    // Set custom apogee velocity threshold
    config.withApogeeVelocityThreshold(1.5);

    TEST_ASSERT_EQUAL_DOUBLE(1.5, config.getApogeeVelocityThreshold());
}

void test_config_with_landing_velocity_threshold() {
    AstraRocketConfig config;

    // Set custom landing velocity threshold
    config.withLandingVelocityThreshold(0.5);

    TEST_ASSERT_EQUAL_DOUBLE(0.5, config.getLandingVelocityThreshold());
}

void test_config_with_landing_duration() {
    AstraRocketConfig config;

    // Set custom landing detection duration
    config.withLandingDetectDuration(5000);

    TEST_ASSERT_EQUAL_UINT32(5000, config.getLandingDetectDuration());
}

void test_config_with_sd_card_cs() {
    AstraRocketConfig config;

    // Set SD card CS pin
    config.withSDCardCS(10);

    TEST_ASSERT_EQUAL_INT(10, config.getSDCardCS());
}

void test_config_with_preflight_log_rate() {
    AstraRocketConfig config;

    // Set preflight log rate
    config.withPreflightLogRate(2.0);

    TEST_ASSERT_EQUAL_DOUBLE(2.0, config.getPreflightLogRate());
}

void test_config_with_flight_log_rate() {
    AstraRocketConfig config;

    // Set flight log rate
    config.withFlightLogRate(100.0);

    TEST_ASSERT_EQUAL_DOUBLE(100.0, config.getFlightLogRate());
}

void test_config_with_postflight_log_rate() {
    AstraRocketConfig config;

    // Set postflight log rate
    config.withPostflightLogRate(0.5);

    TEST_ASSERT_EQUAL_DOUBLE(0.5, config.getPostflightLogRate());
}

void test_config_with_flash_backup() {
    AstraRocketConfig config;

    // Enable flash backup
    config.withFlashBackup(true);

    TEST_ASSERT_TRUE(config.getFlashBackup());

    // Disable flash backup
    config.withFlashBackup(false);

    TEST_ASSERT_FALSE(config.getFlashBackup());
}

void test_config_builder_chaining() {
    AstraRocketConfig config;

    // Test that methods can be chained
    config.withLiftoffAccelThreshold(5.0)
          .withFlightLogRate(75.0);

    // Verify all settings applied
    TEST_ASSERT_EQUAL_DOUBLE(5.0, config.getLiftoffAccelThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(75.0, config.getFlightLogRate());
}

void test_config_all_flight_thresholds() {
    AstraRocketConfig config;

    // Set all flight detection thresholds
    config.withLiftoffAccelThreshold(4.0)
          .withLiftoffDetectDuration(150)
          .withBurnoutAccelThreshold(1.8)
          .withApogeeVelocityThreshold(1.0)
          .withLandingVelocityThreshold(0.8)
          .withLandingDetectDuration(2500);

    TEST_ASSERT_EQUAL_DOUBLE(4.0, config.getLiftoffAccelThreshold());
    TEST_ASSERT_EQUAL_UINT32(150, config.getLiftoffDetectDuration());
    TEST_ASSERT_EQUAL_DOUBLE(1.8, config.getBurnoutAccelThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(1.0, config.getApogeeVelocityThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(0.8, config.getLandingVelocityThreshold());
    TEST_ASSERT_EQUAL_UINT32(2500, config.getLandingDetectDuration());
}

void test_config_all_logging_settings() {
    AstraRocketConfig config;

    // Set all logging settings
    config.withSDCardCS(15)
          .withPreflightLogRate(0.5)
          .withFlightLogRate(200.0)
          .withPostflightLogRate(2.0)
          .withFlashBackup(true);

    TEST_ASSERT_EQUAL_INT(15, config.getSDCardCS());
    TEST_ASSERT_EQUAL_DOUBLE(0.5, config.getPreflightLogRate());
    TEST_ASSERT_EQUAL_DOUBLE(200.0, config.getFlightLogRate());
    TEST_ASSERT_EQUAL_DOUBLE(2.0, config.getPostflightLogRate());
    TEST_ASSERT_TRUE(config.getFlashBackup());
}

void test_config_astra_config_access() {
    AstraRocketConfig config;

    // Verify that AstraRocketConfig inherits from AstraConfig
    // We can now use AstraConfig methods directly on the config object
    AstraConfig* astraConfig = &config;
    TEST_ASSERT_NOT_NULL(astraConfig);
}

void test_config_complex_scenario() {
    AstraRocketConfig config;

    // Configure for a high-power rocket with custom settings
    config.withLiftoffAccelThreshold(6.0)          // High-thrust motor
          .withLiftoffDetectDuration(50)           // Faster detection
          .withBurnoutAccelThreshold(2.5)          // Higher burnout threshold
          .withApogeeVelocityThreshold(3.0)        // Higher threshold for high altitude
          .withFlightLogRate(100.0)                // High-rate logging
          .withSDCardCS(10);

    // Verify complex configuration
    TEST_ASSERT_EQUAL_DOUBLE(6.0, config.getLiftoffAccelThreshold());
    TEST_ASSERT_EQUAL_UINT32(50, config.getLiftoffDetectDuration());
    TEST_ASSERT_EQUAL_DOUBLE(2.5, config.getBurnoutAccelThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(3.0, config.getApogeeVelocityThreshold());
    TEST_ASSERT_EQUAL_DOUBLE(100.0, config.getFlightLogRate());
    TEST_ASSERT_EQUAL_INT(10, config.getSDCardCS());
}

// ---

// This is the main function that runs all the tests. It should be the last thing in the file.
int main(int argc, char **argv)
{
    UNITY_BEGIN();

    // Add your tests here
    RUN_TEST(test_config_default_values);
    RUN_TEST(test_config_with_liftoff_threshold);
    RUN_TEST(test_config_with_liftoff_duration);
    RUN_TEST(test_config_with_burnout_threshold);
    RUN_TEST(test_config_with_apogee_velocity_threshold);
    RUN_TEST(test_config_with_landing_velocity_threshold);
    RUN_TEST(test_config_with_landing_duration);
    RUN_TEST(test_config_with_sd_card_cs);
    RUN_TEST(test_config_with_preflight_log_rate);
    RUN_TEST(test_config_with_flight_log_rate);
    RUN_TEST(test_config_with_postflight_log_rate);
    RUN_TEST(test_config_with_flash_backup);
    RUN_TEST(test_config_builder_chaining);
    RUN_TEST(test_config_all_flight_thresholds);
    RUN_TEST(test_config_all_logging_settings);
    RUN_TEST(test_config_astra_config_access);
    RUN_TEST(test_config_complex_scenario);

    UNITY_END();
}
// ---

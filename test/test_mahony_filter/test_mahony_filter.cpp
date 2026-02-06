#include <unity.h>
#include <NativeTestHelper.h>
#include <UnitTestSensors.h>
#include "../../../.pio/libdeps/native/TRT-Astra/src/Filters/Mahony.h"
#include "../../../.pio/libdeps/native/TRT-Astra/src/Math/Vector.h"
#include <cmath>

using namespace astra;

// Test fixtures
MahonyAHRS* mahony;
FakeIMU fakeIMU;

void setUp(void) {
    fakeIMU.init();
    mahony = new MahonyAHRS(0.1, 0.0005);  // Kp=0.1, Ki=0.0005
}

void tearDown(void) {
    delete mahony;
    mahony = nullptr;
}

/**
 * Test 1: Stationary on pad - should align to gravity
 */
void test_stationary_alignment() {
    // Simulate sensor readings: stationary with Z-axis up
    Vector<3> accel(0, 0, -9.81);  // Gravity down Z
    Vector<3> gyro(0, 0, 0);        // No rotation
    Vector<3> mag(0, 25, 0);        // North pointing (arbitrary)

    // Update filter for 1 second (50 samples at 50Hz)
    for (int i = 0; i < 50; i++) {
        mahony->update(accel, gyro, 0.02);  // dt = 20ms
    }

    TEST_ASSERT_TRUE(mahony->isReady());

    Quaternion q = mahony->getQuaternion();

    // Should be near identity (no rotation from level)
    TEST_ASSERT_FLOAT_WITHIN(0.1, 1.0, q.w());
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, q.x());
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, q.y());
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, q.z());
}

/**
 * Test 2: Constant rotation - should track angular position
 */
void test_constant_rotation() {
    // Initialize stationary
    Vector<3> accel(0, 0, -9.81);
    Vector<3> gyro(0, 0, 0);
    Vector<3> mag(0, 25, 0);

    for (int i = 0; i < 50; i++) {
        mahony->update(accel, gyro, 0.02);
    }

    // Continue with default filter behavior (no frame locking)

    // Now apply constant rotation around Z-axis: 90 deg/s = π/2 rad/s
    gyro = Vector<3>(0, 0, M_PI / 2.0);

    // Rotate for 2 seconds (should be 180 degrees)
    for (int i = 0; i < 100; i++) {
        mahony->update(accel, gyro, 0.02);
    }

    Quaternion q = mahony->getQuaternion();

    // After 180 degree rotation around Z, expect significant Z component
    TEST_ASSERT_TRUE(fabs(q.z()) > 0.5);
}

/**
 * Test 3: Tilt detection - should measure pitch correctly
 */
void test_tilt_detection() {
    // Initialize level
    Vector<3> accel(0, 0, -9.81);
    Vector<3> gyro(0, 0, 0);
    Vector<3> mag(0, 25, 0);

    for (int i = 0; i < 50; i++) {
        mahony->update(accel, gyro, 0.02);
    }

    // Continue with default filter behavior (no frame locking)

    // Tilt 45 degrees forward (pitch)
    // Acceleration vector rotates: a = [sin(45°)*g, 0, -cos(45°)*g]
    double angle_rad = M_PI / 4.0;  // 45 degrees
    accel = Vector<3>(sin(angle_rad) * 9.81, 0, -cos(angle_rad) * 9.81);

    // Let filter converge to tilted orientation
    for (int i = 0; i < 100; i++) {
        mahony->update(accel, gyro, 0.02);
    }

    Quaternion q = mahony->getQuaternion();

    // Verify quaternion indicates pitch rotation
    // For 45° pitch around Y-axis: q ≈ [cos(22.5°), 0, sin(22.5°), 0]
    TEST_ASSERT_TRUE(fabs(q.y()) > 0.3);  // Should have significant Y component
}

/**
 * Test 4: Live sensor test with serial output
 * This function prints sensor data and orientation for live visualization
 */
void test_live_sensor_stream() {
    // This test simulates a complete flight scenario and outputs data
    // for the Python visualization script

    printf("# Live Mahony Filter Test\n");
    printf("# Format: time,ax,ay,az,gx,gy,gz,mx,my,mz,qw,qx,qy,qz,roll,pitch,yaw\n");

    double time = 0.0;
    double dt = 0.02;  // 50 Hz

    // Phase 1: Stationary on pad (0-2s)
    Vector<3> accel(0, 0, -9.81);
    Vector<3> gyro(0, 0, 0);
    Vector<3> mag(0, 25, 0);

    for (int i = 0; i < 100; i++) {
        mahony->update(accel, gyro, dt);

        Quaternion q = mahony->getQuaternion();

        // Convert quaternion to Euler angles (roll, pitch, yaw)
        double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
        double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.4f,%0.4f,%0.4f,%0.4f,%0.3f,%0.3f,%0.3f\n",
               time,
               accel.x(), accel.y(), accel.z(),
               gyro.x(), gyro.y(), gyro.z(),
               mag.x(), mag.y(), mag.z(),
               q.w(), q.x(), q.y(), q.z(),
               roll * 180.0 / M_PI,
               pitch * 180.0 / M_PI,
               yaw * 180.0 / M_PI);

        time += dt;
    }

    // Continue with default filter behavior (no frame locking)

    // Phase 2: Liftoff with rotation (2-4s)
    // Simulate rocket spinning and tilting during ascent
    for (int i = 0; i < 100; i++) {
        double t = i * dt;

        // Add rotation around Z (spin)
        gyro = Vector<3>(0, 0, 2.0 * M_PI);  // 360 deg/s spin

        // Add slight tilt
        double tilt_angle = (M_PI / 6.0) * sin(t);  // ±30 degrees oscillation
        accel = Vector<3>(sin(tilt_angle) * 9.81, 0, -cos(tilt_angle) * 9.81);

        mahony->update(accel, gyro, dt);

        Quaternion q = mahony->getQuaternion();

        double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
        double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.4f,%0.4f,%0.4f,%0.4f,%0.3f,%0.3f,%0.3f\n",
               time,
               accel.x(), accel.y(), accel.z(),
               gyro.x(), gyro.y(), gyro.z(),
               mag.x(), mag.y(), mag.z(),
               q.w(), q.x(), q.y(), q.z(),
               roll * 180.0 / M_PI,
               pitch * 180.0 / M_PI,
               yaw * 180.0 / M_PI);

        time += dt;
    }

    // Phase 3: Apogee tumble (4-6s)
    // Simulate random tumbling at apogee
    for (int i = 0; i < 100; i++) {
        double t = i * dt;

        // Random rotations
        gyro = Vector<3>(
            M_PI * sin(t * 2.0),
            M_PI * cos(t * 1.5),
            M_PI * sin(t * 3.0)
        );

        accel = Vector<3>(
            3.0 * sin(t * 2.0),
            3.0 * cos(t * 1.5),
            -9.81 + 3.0 * sin(t)
        );

        mahony->update(accel, gyro, dt);

        Quaternion q = mahony->getQuaternion();

        double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
        double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.4f,%0.4f,%0.4f,%0.4f,%0.3f,%0.3f,%0.3f\n",
               time,
               accel.x(), accel.y(), accel.z(),
               gyro.x(), gyro.y(), gyro.z(),
               mag.x(), mag.y(), mag.z(),
               q.w(), q.x(), q.y(), q.z(),
               roll * 180.0 / M_PI,
               pitch * 180.0 / M_PI,
               yaw * 180.0 / M_PI);

        time += dt;
    }

    // Phase 4: Stable descent (6-8s)
    gyro = Vector<3>(0, 0, 0.5);  // Slow spin
    accel = Vector<3>(0, 0, -9.81);

    for (int i = 0; i < 100; i++) {
        mahony->update(accel, gyro, dt);

        Quaternion q = mahony->getQuaternion();

        double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
        double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        printf("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.4f,%0.4f,%0.4f,%0.4f,%0.3f,%0.3f,%0.3f\n",
               time,
               accel.x(), accel.y(), accel.z(),
               gyro.x(), gyro.y(), gyro.z(),
               mag.x(), mag.y(), mag.z(),
               q.w(), q.x(), q.y(), q.z(),
               roll * 180.0 / M_PI,
               pitch * 180.0 / M_PI,
               yaw * 180.0 / M_PI);

        time += dt;
    }

    TEST_ASSERT_TRUE(true);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_stationary_alignment);
    RUN_TEST(test_constant_rotation);
    RUN_TEST(test_tilt_detection);
    RUN_TEST(test_live_sensor_stream);

    UNITY_END();
}

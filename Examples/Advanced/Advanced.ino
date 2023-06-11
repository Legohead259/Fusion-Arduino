#include "../../Fusion/Fusion.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <Arduino.h>

#define SAMPLE_RATE (100) // replace this with actual sample rate

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for serial port connection to open

    Serial.println("|-------------|");
    Serial.println("|Fusion Tester|");
    Serial.println("|-------------|");

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}

void loop() {
    // Acquire latest sensor data
    uint32_t timestamp = micros(); // replace this with actual gyroscope timestamp
    FusionVector gyroscope = {0.0f, 0.0f, 0.0f}; // replace this with actual gyroscope data in degrees/s
    FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
    FusionVector magnetometer = {1.0f, 0.0f, 0.0f}; // replace this with actual magnetometer data in arbitrary units

    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    static uint32_t previousTimestamp;
    const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
    previousTimestamp = timestamp;

    // Update gyroscope AHRS algorithm
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

    // Print algorithm outputs
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

    Serial.printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
            euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
            earth.axis.x, earth.axis.y, earth.axis.z);

    delay(SAMPLE_RATE);
}
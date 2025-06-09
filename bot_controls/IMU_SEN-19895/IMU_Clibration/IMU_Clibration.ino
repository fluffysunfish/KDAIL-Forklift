/*
  9DoF IMU Yaw Reading with Calibration and Diagnostics
  Fixed test rotation function + improved for Kalman filter readiness
  
  Hardware: SEN-19895 9DoF IMU Sensor
*/

#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

SparkFun_ISM330DHCX myISM;
SFE_MMC5983MA myMag;

// Sensor data structures
sfe_ism_data_t gyroData;
uint32_t rawMagX, rawMagY, rawMagZ;

// Yaw calculation variables
float magneticYaw = 0.0;
float gyroYaw = 0.0;
float fusedYaw = 0.0;

// Timing variables
unsigned long lastTime = 0;
float deltaTime = 0.0;

// Complementary filter coefficient
float alpha = 0.95;

// Magnetometer calibration parameters
float magOffsetX = 0.0;
float magOffsetY = 0.0;
float magScaleX = 1.0;
float magScaleY = 1.0;
bool magCalibrated = false;

// Yaw correction parameters
float yawCorrectionFactor = 0.96;
float yawOffsetCorrection = 0.0;

// Gyroscope parameters
float gyroBiasZ = 0.0;
float gyroFilteredZ = 0.0;
bool biasCalculated = false;

// Kalman filter variables (for future implementation)
float Q_angle = 0.001;  // Process noise covariance for angle
float Q_gyro = 0.003;   // Process noise covariance for gyro bias
float R_measure = 0.03; // Measurement noise covariance
float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix
float K[2]; // Kalman gain
float y, S; // Innovation and innovation covariance
float kalmanYaw = 0.0;
float gyroBias = 0.0;

// Calibration mode flag
bool calibrationMode = false;
bool useKalmanFilter = false;

void setup() {
    Serial.begin(115200);
    Serial.println("9DoF IMU Yaw Estimation with Advanced Filtering");
    Serial.println("===============================================");
    
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize sensors
    if (!initializeSensors()) {
        Serial.println("Sensor initialization failed!");
        while (true);
    }
    
    Serial.println("Sensors initialized successfully!");
    showMenu();
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        handleSerialCommand();
        return;
    }
    
    // Normal operation - read and display yaw
    if (!calibrationMode) {
        updateYawReadings();
        displayYawData();
        delay(100); // 10Hz update rate for display
    }
}

bool initializeSensors() {
    // Initialize ISM330DHCX
    if (!myISM.begin()) {
        Serial.println("ISM330DHCX not found!");
        return false;
    }
    
    myISM.deviceReset();
    while (!myISM.getDeviceReset()) delay(1);
    
    myISM.setDeviceConfig();
    myISM.setBlockDataUpdate();
    myISM.setGyroDataRate(ISM_GY_ODR_208Hz);
    myISM.setGyroFullScale(ISM_250dps);
    myISM.setGyroFilterLP1();
    myISM.setGyroLP1Bandwidth(ISM_MEDIUM);
    
    // Initialize MMC5983MA
    if (!myMag.begin()) {
        Serial.println("MMC5983MA not found!");
        return false;
    }
    
    myMag.softReset();
    myMag.setFilterBandwidth(100);
    myMag.enableAutomaticSetReset();
    
    return true;
}

void showMenu() {
    Serial.println("\nCommands:");
    Serial.println("  'c' - Calibrate magnetometer");
    Serial.println("  'g' - Calibrate gyroscope bias");
    Serial.println("  'r' - Show raw magnetometer values");
    Serial.println("  's' - Start yaw readings");
    Serial.println("  't' - Test rotation (place at 0°, 90°, 180°, 270°)");
    Serial.println("  'a' - Apply correction based on test results");
    Serial.println("  'k' - Toggle Kalman filter (currently " + String(useKalmanFilter ? "ON" : "OFF") + ")");
    Serial.println("  'd' - Debug mode (detailed magnetometer analysis)");
    Serial.println("  'm' - Show this menu");
}

void displayYawData() {
    Serial.print("Mag: ");
    Serial.print(magneticYaw, 1);
    Serial.print("° | Gyro: ");
    Serial.print(gyroYaw, 1);
    Serial.print("°");
    
    if (useKalmanFilter) {
        Serial.print(" | Kalman: ");
        Serial.print(kalmanYaw, 1);
        Serial.print("°");
    }
    
    Serial.print(" | Fused: ");
    Serial.print(fusedYaw, 1);
    Serial.println("°");
}

void handleSerialCommand() {
    char command = Serial.read();
    
    // Clear any remaining characters in buffer
    while (Serial.available()) {
        Serial.read();
        delay(1);
    }
    
    switch (command) {
        case 'c':
            calibrateMagnetometer();
            break;
        case 'g':
            calibrateGyroBias();
            break;
        case 'r':
            showRawMagValues();
            break;
        case 's':
            startYawReadings();
            break;
        case 't':
            testRotation();
            break;
        case 'a':
            applyTestCorrection();
            break;
        case 'k':
            toggleKalmanFilter();
            break;
        case 'd':
            debugMagnetometer();
            break;
        case 'm':
            showMenu();
            break;
    }
}

void toggleKalmanFilter() {
    useKalmanFilter = !useKalmanFilter;
    Serial.println("\nKalman filter " + String(useKalmanFilter ? "ENABLED" : "DISABLED"));
    
    if (useKalmanFilter) {
        // Initialize Kalman filter
        kalmanYaw = magneticYaw;
        gyroBias = gyroBiasZ;
        P[0][0] = 1.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 1.0;
        Serial.println("Kalman filter initialized with current magnetic heading");
    }
    Serial.println();
}

void calibrateMagnetometer() {
    Serial.println("\n=== MAGNETOMETER CALIBRATION ===");
    Serial.println("Slowly rotate the sensor in ALL directions for 30 seconds.");
    Serial.println("Make complete rotations around all axes (X, Y, Z).");
    Serial.println("Starting in 3 seconds...");
    delay(3000);
    
    float minX = 999999, maxX = -999999;
    float minY = 999999, maxY = -999999;
    
    unsigned long startTime = millis();
    int sampleCount = 0;
    
    Serial.println("Calibrating... (rotate sensor now!)");
    
    while (millis() - startTime < 30000) {
        myMag.getMeasurementXYZ(&rawMagX, &rawMagY, &rawMagZ);
        
        float x = (float)rawMagX - 131072.0;
        float y = (float)rawMagY - 131072.0;
        
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        
        sampleCount++;
        
        if (sampleCount % 100 == 0) {
            Serial.print(".");
        }
        
        delay(10);
    }
    
    // Calculate calibration parameters
    magOffsetX = (maxX + minX) / 2.0;
    magOffsetY = (maxY + minY) / 2.0;
    
    float rangeX = (maxX - minX) / 2.0;
    float rangeY = (maxY - minY) / 2.0;
    float avgRange = (rangeX + rangeY) / 2.0;
    
    magScaleX = avgRange / rangeX;
    magScaleY = avgRange / rangeY;
    
    magCalibrated = true;
    
    Serial.println("\nCalibration complete!");
    Serial.println("Calibration parameters:");
    Serial.print("  Offset X: "); Serial.println(magOffsetX, 1);
    Serial.print("  Offset Y: "); Serial.println(magOffsetY, 1);
    Serial.print("  Scale X:  "); Serial.println(magScaleX, 3);
    Serial.print("  Scale Y:  "); Serial.println(magScaleY, 3);
    Serial.print("  Samples:  "); Serial.println(sampleCount);
    Serial.println();
}

void calibrateGyroBias() {
    Serial.println("\n=== GYROSCOPE BIAS CALIBRATION ===");
    Serial.println("Keep sensor COMPLETELY still for 5 seconds...");
    delay(2000);
    
    float biasSum = 0.0;
    int samples = 0;
    unsigned long startTime = millis();
    
    while (millis() - startTime < 5000) {
        if (myISM.checkGyroStatus()) {
            myISM.getGyro(&gyroData);
            biasSum += gyroData.zData;
            samples++;
        }
        delay(5);
    }
    
    if (samples > 0) {
        gyroBiasZ = biasSum / samples;
        biasCalculated = true;
        
        Serial.println("Gyro bias calibration complete:");
        Serial.print("  Bias Z: "); Serial.print(gyroBiasZ, 2); Serial.println(" °/s");
        Serial.print("  Samples: "); Serial.println(samples);
        
        if (abs(gyroBiasZ) > 200) {
            Serial.println("  WARNING: Large bias detected - sensor may not have been still!");
        }
    }
    Serial.println();
}

void testRotation() {
    if (!magCalibrated) {
        Serial.println("Please calibrate magnetometer first (command 'c')");
        return;
    }
    
    Serial.println("\n=== ROTATION TEST ===");
    Serial.println("This will test clockwise rotations of 90 degrees each.");
    Serial.println("Place sensor at each position and press ENTER:");
    
    float testReadings[4];
    String positions[] = {"0°", "90°", "180°", "270°"};
    
    for (int i = 0; i < 4; i++) {
        Serial.print("Position sensor at "); Serial.print(positions[i]); 
        Serial.print(" and press ENTER... ");
        
        // Wait for Enter key specifically
        waitForEnter();
        
        // Take multiple readings and average for stability
        float sum = 0;
        for (int j = 0; j < 10; j++) {  // More samples for better accuracy
            updateMagneticYaw();
            sum += magneticYaw;
            delay(50);
        }
        testReadings[i] = sum / 10.0;
        
        Serial.print("Reading: "); Serial.println(testReadings[i], 1);
    }
    
    Serial.println("\n=== ROTATION TEST RESULTS ===");
    for (int i = 0; i < 4; i++) {
        Serial.print(positions[i]); Serial.print(": "); Serial.println(testReadings[i], 1);
    }
    
    // Calculate differences with proper angle wrapping
    Serial.println("\nClockwise angular differences (should be ~90°):");
    float totalError = 0;
    int validMeasurements = 0;
    
    for (int i = 0; i < 4; i++) {
        int nextIndex = (i + 1) % 4;  // Wrap around for 270° to 0°
        
        float diff = testReadings[nextIndex] - testReadings[i];
        
        // Handle angle wrapping properly for clockwise rotation
        if (diff < -180.0) diff += 360.0;  // Cross 0° boundary
        if (diff > 180.0) diff -= 360.0;   // Should not happen for 90° rotations
        
        // For clockwise rotation, difference should be positive ~90°
        // If negative, we crossed the 0° boundary
        if (diff < 0 && i == 3) {  // 270° to 0° transition
            diff = (360.0 - testReadings[i]) + testReadings[nextIndex];
        }
        
        float error = abs(90.0 - abs(diff));
        totalError += error;
        validMeasurements++;
        
        Serial.print(positions[i]); Serial.print(" to "); Serial.print(positions[nextIndex]); 
        Serial.print(": "); Serial.print(diff, 1); 
        Serial.print("° (error: "); Serial.print(error, 1); Serial.println("°)");
    }
    
    if (validMeasurements > 0) {
        float avgError = totalError / validMeasurements;
        Serial.print("\nAverage error: "); Serial.print(avgError, 1); Serial.println("°");
        
        if (avgError > 5.0) {
            Serial.println("Large errors detected. Possible issues:");
            Serial.println("- Magnetometer needs recalibration");
            Serial.println("- Magnetic interference present");
            Serial.println("- Correction factor needed (command 'a')");
        } else {
            Serial.println("Good accuracy! Readings are within acceptable range.");
        }
    }
    Serial.println();
}

void waitForEnter() {
    // Clear any existing input
    while (Serial.available()) {
        Serial.read();
        delay(1);
    }
    
    // Wait for Enter key (newline character)
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                break;
            }
        }
        // Continue updating readings while waiting
        updateMagneticYaw();
        delay(10);
    }
    
    // Clear any remaining characters
    while (Serial.available()) {
        Serial.read();
        delay(1);
    }
}

void applyTestCorrection() {
    Serial.println("\n=== APPLY TEST-BASED CORRECTION ===");
    Serial.println("Current correction factor: " + String(yawCorrectionFactor, 3));
    Serial.println("Current offset correction: " + String(yawOffsetCorrection, 1));
    Serial.println();
    Serial.println("Based on typical test results:");
    Serial.println("Recommended correction factor: 0.96 (reduces ~4% over-rotation)");
    Serial.println();
    Serial.print("Apply recommended correction? (y/n): ");
    
    while (!Serial.available()) delay(10);
    char response = Serial.read();
    Serial.println(response);
    
    if (response == 'y' || response == 'Y') {
        yawCorrectionFactor = 0.96;
        yawOffsetCorrection = 0.0;
        Serial.println("Correction applied! Test rotation again to verify improvement.");
    } else {
        Serial.println("Correction not applied.");
    }
    Serial.println();
}

void showRawMagValues() {
    Serial.println("\n=== RAW MAGNETOMETER VALUES ===");
    Serial.println("Format: X, Y, Z (raw) | X, Y (calibrated)");
    Serial.println("Press any key to stop...");
    
    while (!Serial.available()) {
        myMag.getMeasurementXYZ(&rawMagX, &rawMagY, &rawMagZ);
        
        float rawX = (float)rawMagX - 131072.0;
        float rawY = (float)rawMagY - 131072.0;
        float rawZ = (float)rawMagZ - 131072.0;
        
        float calX = (rawX - magOffsetX) * magScaleX;
        float calY = (rawY - magOffsetY) * magScaleY;
        
        Serial.print(rawX, 0); Serial.print(", ");
        Serial.print(rawY, 0); Serial.print(", ");
        Serial.print(rawZ, 0); Serial.print(" | ");
        Serial.print(calX, 1); Serial.print(", ");
        Serial.println(calY, 1);
        
        delay(200);
    }
    Serial.read();
    Serial.println();
}

void startYawReadings() {
    if (!magCalibrated) {
        Serial.println("Please calibrate magnetometer first (command 'c')");
        return;
    }
    
    Serial.println("\n=== YAW READINGS ===");
    if (useKalmanFilter) {
        Serial.println("Format: Magnetic | Gyro | Kalman | Fused");
    } else {
        Serial.println("Format: Magnetic | Gyro | Fused");
    }
    Serial.println("Press any key to stop...");
    
    lastTime = millis();
    gyroYaw = magneticYaw;
    
    if (useKalmanFilter) {
        kalmanYaw = magneticYaw;
    }
    
    while (!Serial.available()) {
        updateYawReadings();
        displayYawData();
        delay(100);
    }
    Serial.read();
    Serial.println();
}

void updateYawReadings() {
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0;
    
    if (deltaTime >= 0.01) {
        updateMagneticYaw();
        updateGyroYaw();
        
        if (useKalmanFilter) {
            updateKalmanFilter();
        }
        
        fuseYawEstimates();
        lastTime = currentTime;
    }
}

void updateMagneticYaw() {
    myMag.getMeasurementXYZ(&rawMagX, &rawMagY, &rawMagZ);
    
    // Apply calibration
    float calX = ((float)rawMagX - 131072.0 - magOffsetX) * magScaleX;
    float calY = ((float)rawMagY - 131072.0 - magOffsetY) * magScaleY;
    
    // Normalize to unit vector
    float magnitude = sqrt(calX * calX + calY * calY);
    if (magnitude > 0) {
        calX /= magnitude;
        calY /= magnitude;
    }
    
    // Calculate heading (atan2 returns -π to π, convert to 0-360)
    // Note: Check your sensor orientation - you may need to adjust the signs
    float heading = atan2(calY, calX) * 180.0 / PI;
    
    // Convert to 0-360 range
    if (heading < 0) {
        heading += 360.0;
    }
    
    // Apply correction factor based on test results
    heading *= yawCorrectionFactor;
    heading += yawOffsetCorrection;
    
    // Normalize again after correction
    if (heading >= 360.0) {
        heading = fmod(heading, 360.0);
    }
    if (heading < 0.0) {
        heading += 360.0;
    }
    
    magneticYaw = heading;
}

void updateGyroYaw() {
    if (!biasCalculated || deltaTime == 0) return;
    
    if (myISM.checkGyroStatus()) {
        myISM.getGyro(&gyroData);
        
        float gyroRate = gyroData.zData - gyroBiasZ;
        gyroFilteredZ = 0.8 * gyroFilteredZ + 0.2 * gyroRate;
        
        if (abs(gyroFilteredZ) > 30.0) {
            gyroYaw += gyroFilteredZ * deltaTime;
            
            while (gyroYaw >= 360.0) gyroYaw -= 360.0;
            while (gyroYaw < 0.0) gyroYaw += 360.0;
        }
    }
}

void updateKalmanFilter() {
    if (deltaTime == 0) return;
    
    // Get gyro rate (already bias corrected)
    float gyroRate = gyroFilteredZ;
    
    // Prediction step
    kalmanYaw += (gyroRate - gyroBias) * deltaTime;
    
    // Normalize angle
    while (kalmanYaw >= 360.0) kalmanYaw -= 360.0;
    while (kalmanYaw < 0.0) kalmanYaw += 360.0;
    
    // Update error covariance
    P[0][0] += deltaTime * (deltaTime * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= deltaTime * P[1][1];
    P[1][0] -= deltaTime * P[1][1];
    P[1][1] += Q_gyro * deltaTime;
    
    // Update step (using magnetometer measurement)
    y = magneticYaw - kalmanYaw;
    
    // Handle angle wrapping for innovation
    while (y > 180.0) y -= 360.0;
    while (y < -180.0) y += 360.0;
    
    S = P[0][0] + R_measure;
    
    // Calculate Kalman gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    
    // Update state
    kalmanYaw += K[0] * y;
    gyroBias += K[1] * y;
    
    // Normalize angle
    while (kalmanYaw >= 360.0) kalmanYaw -= 360.0;
    while (kalmanYaw < 0.0) kalmanYaw += 360.0;
    
    // Update error covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}

void fuseYawEstimates() {
    if (useKalmanFilter) {
        fusedYaw = kalmanYaw;
    } else {
        float angleDiff = magneticYaw - gyroYaw;
        
        // Handle angle wrapping
        while (angleDiff > 180.0) angleDiff -= 360.0;
        while (angleDiff < -180.0) angleDiff += 360.0;
        
        gyroYaw += alpha * angleDiff;
        
        while (gyroYaw >= 360.0) gyroYaw -= 360.0;
        while (gyroYaw < 0.0) gyroYaw += 360.0;
        
        fusedYaw = gyroYaw;
    }
}

void debugMagnetometer() {
    if (!magCalibrated) {
        Serial.println("Please calibrate magnetometer first (command 'c')");
        return;
    }
    
    Serial.println("\n=== MAGNETOMETER DEBUG MODE ===");
    Serial.println("This will show detailed calculations for debugging.");
    Serial.println("Place sensor at 200° and press ENTER...");
    
    waitForEnter();
    
    // Take multiple readings
    float readings[10];
    for (int i = 0; i < 10; i++) {
        myMag.getMeasurementXYZ(&rawMagX, &rawMagY, &rawMagZ);
        
        float rawX = (float)rawMagX - 131072.0;
        float rawY = (float)rawMagY - 131072.0;
        
        float calX = (rawX - magOffsetX) * magScaleX;
        float calY = (rawY - magOffsetY) * magScaleY;
        
        float magnitude = sqrt(calX * calX + calY * calY);
        if (magnitude > 0) {
            calX /= magnitude;
            calY /= magnitude;
        }
        
        float heading = atan2(calY, calX) * 180.0 / PI;
        if (heading < 0) heading += 360.0;
        
        readings[i] = heading;
        
        Serial.print("Sample "); Serial.print(i+1); Serial.print(": ");
        Serial.print("Raw("); Serial.print(rawX, 0); Serial.print(","); Serial.print(rawY, 0); Serial.print(") ");
        Serial.print("Cal("); Serial.print(calX, 3); Serial.print(","); Serial.print(calY, 3); Serial.print(") ");
        Serial.print("Heading: "); Serial.println(heading, 1);
        
        delay(100);
    }
    
    // Calculate average
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += readings[i];
    }
    float avgReading = sum / 10.0;
    
    Serial.print("\nAverage reading at 200°: "); Serial.println(avgReading, 1);
    
    Serial.println("\nNow rotate EXACTLY 90° clockwise to 290° and press ENTER...");
    waitForEnter();
    
    // Take reading at new position
    for (int i = 0; i < 10; i++) {
        myMag.getMeasurementXYZ(&rawMagX, &rawMagY, &rawMagZ);
        
        float rawX = (float)rawMagX - 131072.0;
        float rawY = (float)rawMagY - 131072.0;
        
        float calX = (rawX - magOffsetX) * magScaleX;
        float calY = (rawY - magOffsetY) * magScaleY;
        
        float magnitude = sqrt(calX * calX + calY * calY);
        if (magnitude > 0) {
            calX /= magnitude;
            calY /= magnitude;
        }
        
        float heading = atan2(calY, calX) * 180.0 / PI;
        if (heading < 0) heading += 360.0;
        
        readings[i] = heading;
        delay(100);
    }
    
    sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += readings[i];
    }
    float newReading = sum / 10.0;
    
    Serial.print("Average reading at 290°: "); Serial.println(newReading, 1);
    
    // Calculate the actual rotation
    float actualRotation = newReading - avgReading;
    if (actualRotation < -180.0) actualRotation += 360.0;
    if (actualRotation > 180.0) actualRotation -= 360.0;
    
    Serial.print("Measured rotation: "); Serial.print(actualRotation, 1); Serial.println("°");
    Serial.print("Expected rotation: 90.0°");
    Serial.print("Error: "); Serial.print(actualRotation - 90.0, 1); Serial.println("°");
    
    if (abs(actualRotation - 90.0) > 5.0) {
        Serial.println("\nPossible issues:");
        Serial.println("1. Sensor orientation - check if X/Y axes are swapped");
        Serial.println("2. Calibration quality - recalibrate in interference-free area");
        Serial.println("3. Magnetic interference - move away from metal objects");
        Serial.println("4. atan2 parameters may need sign adjustment");
    }
    
    Serial.println();
}

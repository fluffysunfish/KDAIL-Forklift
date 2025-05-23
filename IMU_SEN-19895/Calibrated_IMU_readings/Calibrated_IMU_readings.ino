/*
  Simplified 9DoF IMU Yaw Reading with Fixed Calibration
  Hardware: SEN-19895 9DoF IMU Sensor
  Outputs: JSON format for easy parsing by Raspberry Pi
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

// Fixed calibration parameters (your values)
const float magOffsetX = -4382.5;
const float magOffsetY = 13239.0;
const float magScaleX = 1.075;
const float magScaleY = 0.935;

// Gyroscope parameters
float gyroBiasZ = 0.0;
bool biasCalculated = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize sensors
    if (!initializeSensors()) {
        Serial.println("{\"error\":\"Sensor initialization failed\"}");
        while (true);
    }
    
    // Calibrate gyroscope bias
    calibrateGyroBias();
    
    Serial.println("{\"status\":\"ready\"}");
    lastTime = millis();
}

void loop() {
    updateYawReadings();
    
    // Output data in JSON format for easy parsing
    Serial.print("{\"magnetic_yaw\":");
    Serial.print(magneticYaw, 2);
    Serial.print(",\"gyro_yaw\":");
    Serial.print(gyroYaw, 2);
    Serial.print(",\"fused_yaw\":");
    Serial.print(fusedYaw, 2);
    Serial.print(",\"timestamp\":");
    Serial.print(millis());
    Serial.println("}");
    
    delay(50); // 20Hz update rate
}

bool initializeSensors() {
    // Initialize ISM330DHCX
    if (!myISM.begin()) {
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
        return false;
    }
    
    myMag.softReset();
    myMag.setFilterBandwidth(100);
    myMag.enableAutomaticSetReset();
    
    return true;
}

void calibrateGyroBias() {
    float biasSum = 0.0;
    int samples = 0;
    unsigned long startTime = millis();
    
    // Collect samples for 3 seconds
    while (millis() - startTime < 3000) {
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
    }
}

void updateYawReadings() {
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0;
    
    if (deltaTime >= 0.02) { // 50Hz max update rate
        updateMagneticYaw();
        updateGyroYaw();
        fuseYawEstimates();
        lastTime = currentTime;
    }
}

void updateMagneticYaw() {
    myMag.getMeasurementXYZ(&rawMagX, &rawMagY, &rawMagZ);
    
    // Apply fixed calibration
    float calX = ((float)rawMagX - 131072.0 - magOffsetX) * magScaleX;
    float calY = ((float)rawMagY - 131072.0 - magOffsetY) * magScaleY;
    
    // Calculate heading
    float heading = atan2(calY, calX) * 180.0 / PI;
    
    // Convert to 0-360 range
    if (heading < 0) {
        heading += 360.0;
    }
    
    magneticYaw = heading;
}

void updateGyroYaw() {
    if (!biasCalculated || deltaTime == 0) return;
    
    if (myISM.checkGyroStatus()) {
        myISM.getGyro(&gyroData);
        
        float gyroRate = gyroData.zData - gyroBiasZ;
        
        // Only integrate if there's significant rotation
        if (abs(gyroRate) > 5.0) {
            gyroYaw += gyroRate * deltaTime;
            
            // Normalize to 0-360 range
            while (gyroYaw >= 360.0) gyroYaw -= 360.0;
            while (gyroYaw < 0.0) gyroYaw += 360.0;
        }
    }
}

void fuseYawEstimates() {
    // Simple complementary filter
    float angleDiff = magneticYaw - gyroYaw;
    
    // Handle angle wrapping
    while (angleDiff > 180.0) angleDiff -= 360.0;
    while (angleDiff < -180.0) angleDiff += 360.0;
    
    // Apply complementary filter
    gyroYaw += alpha * angleDiff;
    
    // Normalize
    while (gyroYaw >= 360.0) gyroYaw -= 360.0;
    while (gyroYaw < 0.0) gyroYaw += 360.0;
    
    fusedYaw = gyroYaw;
}

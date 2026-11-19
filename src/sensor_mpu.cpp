    #include "sensor_mpu.h"
    #include <Wire.h>
    #include <Arduino.h>

    #define MPU_ADDR 0x68
    #define SDA_PIN 21
    #define SCL_PIN 22

    constexpr uint32_t IMU_PERIOD_US = 2500; // 400 Hz
    uint32_t lastUpdate = 0;

    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float roll_acc = 0.0f, pitch_acc = 0.0f;
    float roll = 0.0f, pitch = 0.0f, yaw_g = 0.0f;
    float roll_offset = 0.0f, pitch_offset = 0.0f;
    float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

    void readAccel() {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission((uint8_t)0);
        Wire.requestFrom(MPU_ADDR, 6, true);
        accX = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
        accY = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
        accZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 16384.0f;
    }

    void readGyro() {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x43); Wire.endTransmission((uint8_t)0);
        Wire.requestFrom(MPU_ADDR, 6, true);
        gyroX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
        gyroY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
        gyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0f;
    }

    void computeAccelAngles() {
        // MPU6050 DỰNG ĐỨNG 90°: đã xác định từ dữ liệu thực tế:
        //   - accX ≈ 1.0 khi đứng yên  → X là trục chịu trọng lực (thẳng đứng)
        //   - accY thay đổi khi hất lên/xuống → Y điều khiển Pitch
        //   - accZ thay đổi khi lắc trái/phải → Z điều khiển Roll
        //
        // Công thức tính góc được remap theo đúng trục thực tế:
        //   roll_acc  = góc lắc trái/phải  → dùng accY và accX (thay vì accY và accZ)
        //   pitch_acc = góc hất lên/xuống  → dùng accZ và accY (thay vì accX và accY/Z)
        roll_acc  = atan2(accZ, accX) * 180.0f / PI;
        pitch_acc = atan2(-accY, sqrtf(accZ * accZ + accX * accX)) * 180.0f / PI;
    }

    float wrapAngle(float angle) {
        while (angle >  180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }

    void calibrateGyro() {
        int samples = 500;
        float sumX = 0, sumY = 0, sumZ = 0;
        for (int i = 0; i < samples; i++) {
            readGyro();
            sumX += gyroX; sumY += gyroY; sumZ += gyroZ;
            delay(3);
        }
        gyroBiasX = sumX / samples;
        gyroBiasY = sumY / samples;
        gyroBiasZ = sumZ / samples;
    }

    void initMPU() {
        Wire.setPins(SDA_PIN, SCL_PIN);
        Wire.begin();
        Wire.setClock(400000);
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
        calibrateGyro();
        delay(200);
        readAccel();
        computeAccelAngles();
        roll_offset  = roll_acc;
        pitch_offset = pitch_acc;
        lastUpdate = micros();
    }

    bool updateMPU() {
        uint32_t now = micros();
        // Chỉ tính toán khi thực sự có mẫu cảm biến MỚI (đúng chu kỳ 400Hz).
        // Nếu để void và gọi mỗi vòng loop() như bản cũ, loop() chạy nhanh hơn
        // 400Hz rất nhiều (không delay()) sẽ khiến processMovement() được gọi
        // lặp lại với góc CHƯA đổi nhiều lần liên tiếp -> làm hỏng bộ lọc EMA
        // bên ble_mouse_action.cpp (bị "decay" nhiều lần giữa 2 mẫu thật),
        // gây chuyển động chập chờn, khó điều khiển.
        if (now - lastUpdate < IMU_PERIOD_US) return false;
        float dt = (now - lastUpdate) * 1e-6f;
        lastUpdate = now;

        readAccel();
        readGyro();
        gyroX -= gyroBiasX; gyroY -= gyroBiasY; gyroZ -= gyroBiasZ;
        computeAccelAngles();

        float roll_acc_rel  = wrapAngle(roll_acc  - roll_offset);
        float pitch_acc_rel = wrapAngle(pitch_acc - pitch_offset);

        // Gyro remap theo đúng trục thực tế:
        //   Roll  (lắc trái/phải) → gyroZ thay đổi khi lắc
        //   Pitch (hất lên/xuống) → gyroY thay đổi khi hất  (đã xác nhận từ test)
        //   Yaw   (xoay cổ tay)   → gyroX là trục còn lại
        roll  = wrapAngle(0.98f * (roll  + gyroZ * dt) + 0.02f * roll_acc_rel);
        pitch = wrapAngle(0.98f * (pitch + gyroY * dt) + 0.02f * pitch_acc_rel);
        yaw_g = wrapAngle(yaw_g + gyroX * dt);

        return true;
    }

    float getPitch() { return pitch; }
    float getYaw()   { return yaw_g; }
    float getRoll()  { return roll; }
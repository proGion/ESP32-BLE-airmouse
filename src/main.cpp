#include <Arduino.h>
#include "hardware_io.h"
#include "sensor_mpu.h"
#include "ble_mouse_action.h"

// Tần suất in Serial debug (ms). Để giá trị 0 nếu muốn tắt hẳn khi mạch đã ổn định,
// in Serial mỗi vòng loop sẽ làm chậm vòng lặp không cần thiết.
const unsigned long PRINT_INTERVAL_MS = 50;

void setup() {
    Serial.begin(9600);
    initHardware();
    initBleMouse();
    setLedState(true);
    Serial.println("Đang hiệu chuẩn... Vui lòng đặt chuột đứng yên!");
    initMPU();
    setLedState(false);
    Serial.println("Hệ thống Chuột Bay đã sẵn sàng!");
}

void loop() {
    // Tạm khóa ngủ đông bằng dấu // để test
    // if (checkSleepMode()) return;

    // updateMPU() tự giới hạn tốc độ đọc cảm biến ở 400Hz bằng micros(),
    // không dùng delay() nên không block các nhánh khác của loop.
    // QUAN TRỌNG: chỉ gọi processMovement() khi updateMPU() trả về true
    // (tức vừa có mẫu cảm biến MỚI), để tần số tính toán khớp đúng tần số
    // cảm biến, tránh làm hỏng bộ lọc mượt (EMA) bên trong.
    if (updateMPU()) {
        // FIX TRỤC TỌA ĐỘ: MPU6050 dựng đứng 90° làm trục X/Y hoán đổi nhau.
        // Triệu chứng đã xác nhận: "hất lên/xuống thì chuột chạy ngang"
        // -> Pitch điều khiển trục dọc (Y), Roll điều khiển trục ngang (X).
        //
        // Nếu sau khi nạp code chuột vẫn chạy NGƯỢC CHIỀU tay, chỉnh dấu:
        //   Ngang ngược -> đổi  getRoll()  thành -getRoll()
        //   Dọc ngược   -> đổi -getPitch() thành  getPitch()
        processMovement(getRoll(), -getPitch());
    }

    // handleButtons() dùng debounce non-blocking (millis()), không delay().
    handleButtons();

    // updateBatteryLevel() tự giới hạn bằng millis(), chỉ đọc ADC mỗi 60s.
    updateBatteryLevel();

    // In debug không ảnh hưởng tốc độ vòng lặp vì chỉ chạy theo chu kỳ riêng.
    static unsigned long lastPrint = 0;
    if (PRINT_INTERVAL_MS > 0 && millis() - lastPrint > PRINT_INTERVAL_MS) {
        lastPrint = millis();
        // Debug góc: BẬT để kiểm tra dấu sau khi đảo trục, TẮT khi chạy ổn định.
        // Định dạng: Pitch(ngang)/Roll(doc)
        // Hất tay lên -> Pitch âm là đúng (chuột đi lên màn hình)
        // Lắc tay phải -> Roll dương là đúng (chuột đi sang phải)
        Serial.print("Pitch="); Serial.print(getPitch());
        Serial.print("  Roll="); Serial.println(getRoll());
    }

    // Không có delay() nào trong toàn bộ loop() -> vòng lặp chạy nhanh nhất
    // có thể, các module tự giới hạn tần suất bằng millis()/micros() riêng,
    // giúp nút bấm luôn được polling liên tục dù IMU đang đọc dữ liệu.
}
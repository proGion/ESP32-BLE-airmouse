#include "ble_mouse_action.h"
#include <Arduino.h>
#include <BleMouse.h>
#include <cmath>

BleMouse bleMouse("Air Mouse ESP32", "Team 5", 100);

float lastAngleX = 0.0f;
float lastAngleY = 0.0f;

// --- CẤU HÌNH ĐỘ MƯỢT ---
// --- CẤU HÌNH ĐỘ MƯỢT ---
// sensitivity: nhân trực tiếp với delta góc (độ) để ra pixel/lần gửi.
//   Giá trị cũ 16.0 quá nhanh (chuột trơn trượt, không kiểm soát được).
//   Giảm xuống 4.0 cho cảm giác DPI vừa phải, dễ điều khiển chính xác.
//   Nếu vẫn còn nhanh -> giảm thêm xuống 3.0; chậm quá -> tăng lên 5.0-6.0.
const float sensitivity = 10.0;

// deadzone: bỏ qua delta nhỏ hơn ngưỡng này để lọc rung tay lúc giữ yên.
//
// FIX QUAN TRỌNG: processMovement() giờ chỉ được gọi đúng lúc có mẫu cảm
// biến MỚI, tức đúng chu kỳ 400Hz (~2.5ms/lần) -- xem main.cpp. Ở tần số
// này, delta góc của một chuyển động tay BÌNH THƯỜNG cũng chỉ cỡ 0.05-0.15
// độ/mẫu (ví dụ lắc tay 40°/giây => 40 * 0.0025s = 0.1°/mẫu). Ngưỡng cũ 0.3
// tương đương đòi hỏi tốc độ lắc tay ~120°/giây mới vượt qua được deadzone
// -> gần như MỌI chuyển động bình thường bị xoá về 0 mỗi mẫu, chỉ những lúc
// giật tay thật nhanh mới "lọt", gây cảm giác tốc độ chập chờn, lúc chạy
// lúc đứng, khó điều hướng chính xác.
//
// 0.05 độ/mẫu (~20°/giây) đủ để lọc rung tay sinh lý (vốn rất nhỏ ở chu kỳ
// 2.5ms) nhưng không còn chặn chuyển động tay bình thường.
//   Nếu con trỏ vẫn tự trôi nhẹ khi đặt yên trên bàn -> tăng dần lên 0.06-0.08.
//   Nếu vẫn thấy "ì", phải lắc mạnh mới chạy -> giảm xuống 0.03-0.04.
const float deadzone = 0.1;

// alpha: hệ số EMA (0.1 = rất mượt/trễ, 0.9 = rất nhạy/giật).
//   0.4 là điểm cân bằng tốt: đủ mượt, không cảm giác lag rõ rệt.
const float alpha = 0.4;

float smoothedMoveX = 0;
float smoothedMoveY = 0;

// --- BỘ NHỚ "PHẦN DƯ" (Residual Accumulator) ---
// FIX BUG CHÍNH: khi ép kiểu (int) một số float như 0.9,
// C++ luôn cắt cụt về 0 (không làm tròn) => với tốc độ tay chậm,
// smoothedMoveX dao động quanh dưới 1.0 mãi mà không bao giờ ra lệnh move().
// Đây chính là lý do chuột "kẹt" chỉ đi được 1-2 hướng khi di chậm.
// Giải pháp: giữ lại phần lẻ bị cắt và cộng dồn vào lần tính kế tiếp,
// để không có chuyển động nào bị "ăn mất".
float residualX = 0.0f;
float residualY = 0.0f;

// --- FIX BUG "KHỰNG" CHUỘT: GIỚI HẠN TẦN SỐ GỬI BÁO CÁO BLE ---
// processMovement() được gọi mỗi vòng loop(), và loop() không có delay()
// nên chạy nhanh hơn nhiều so với 400Hz của IMU. Cứ mỗi lần góc thay đổi,
// code cũ gọi bleMouse.move() ngay lập tức -> có thể lên tới ~400 report/giây.
// Kết nối BLE HID thực tế chỉ xử lý kịp khoảng 30-130 report/giây (tùy
// connection interval). Khi hàng đợi notify của BLE stack bị đầy, lệnh
// move() bị nghẽn -> toàn bộ loop() (kể cả đọc nút, đọc cảm biến) bị "khựng"
// theo vì chương trình chạy tuần tự, không có task riêng cho BLE.
//
// Giải pháp: vẫn TÍNH TOÁN (đọc góc, lọc EMA) ở tần số cao để giữ độ mượt,
// nhưng CHỈ GỬI report qua BLE theo chu kỳ cố định, thấp hơn nhiều.
// Trong lúc chờ, phần di chuyển được cộng dồn vào bộ đệm "pending" nên
// không bị mất chuyển động, chỉ là gộp lại rồi gửi 1 lần.
const unsigned long SEND_INTERVAL_MS = 12; // ~83 báo cáo/giây, an toàn cho hầu hết máy
unsigned long lastSendTime = 0;
long pendingX = 0;
long pendingY = 0;

// bleMouse.move() nhận tham số kiểu signed char (int8_t), tức chỉ chứa được
// -127..127. Nếu gửi giá trị vượt ngưỡng này mà không kiểm soát, nó sẽ bị
// tràn số (wrap-around) âm thầm -> chuột giật ngược bất thường.
static inline int8_t clampToInt8(long v) {
    if (v > 127) return 127;
    if (v < -127) return -127;
    return (int8_t)v;
}

// --- FIX BUG "NHẤN NÚT LÀM CHUỘT DỊCH CHUYỂN" ---
// Khi ngón tay chạm/nhả nút, cả bàn tay thường rung/nghiêng nhẹ trong
// khoảnh khắc đó (dù có ý giữ yên). Gyro/accel ghi nhận đây là chuyển động
// THẬT, nên chuột bị "nhảy" đúng lúc bạn định click -- không liên quan gì
// đến deadzone (tăng deadzone chỉ chặn luôn cả chuyển động bình thường,
// gây khựng, chứ không xử lý đúng nguyên nhân).
//
// Giải pháp: hardware_io.cpp sẽ gọi suppressMovementBriefly() ngay tại thời
// điểm phát hiện tín hiệu nút vừa đổi (raw state, trước cả khi debounce xác
// nhận) -- đúng lúc dễ jiggle nhất. Trong SUPPRESS_MS kế tiếp, chuyển động
// bị ép về 0 (nhưng lastAngle vẫn cập nhật bình thường để không bị "giật bù"
// khi hết đóng băng).
const unsigned long SUPPRESS_MS = 50;
unsigned long suppressUntil = 0;

void suppressMovementBriefly() {
    suppressUntil = millis() + SUPPRESS_MS;
}

void initBleMouse() {
    bleMouse.begin();
}

void processMovement(float angleX, float angleY) {
    if (!bleMouse.isConnected()) {
        // Khi mất kết nối: cập nhật lại mốc góc và xoá hết bộ nhớ làm mượt/dư
        // để khi kết nối lại không bị "giật" do dữ liệu cũ tích tụ.
        lastAngleX = angleX;
        lastAngleY = angleY;
        smoothedMoveX = 0;
        smoothedMoveY = 0;
        residualX = 0;
        residualY = 0;
        pendingX = 0;
        pendingY = 0;
        lastSendTime = millis();
        return;
    }

    // 1. Tính toán sự thay đổi góc (Delta)
    float deltaX = angleX - lastAngleX;
    float deltaY = angleY - lastAngleY;

    lastAngleX = angleX;
    lastAngleY = angleY;

    // 2. Lọc bỏ nhiễu rung tay (Deadzone)
    if (fabs(deltaX) < deadzone) deltaX = 0;
    if (fabs(deltaY) < deadzone) deltaY = 0;

    // 2b. Đang trong cửa sổ "đóng băng" do vừa có sự kiện nhấn/nhả nút
    // -> ép về 0 để không ghi nhận rung tay do bấm nút thành chuyển động.
    if (millis() < suppressUntil) {
        deltaX = 0;
        deltaY = 0;
    }

    // 3. Tính toán tốc độ thô
    float rawSpeedX = deltaX * sensitivity;
    float rawSpeedY = deltaY * sensitivity;

    // 4. BỘ LỌC LÀM MƯỢT (Exponential Moving Average)
    smoothedMoveX = (alpha * rawSpeedX) + ((1.0f - alpha) * smoothedMoveX);
    smoothedMoveY = (alpha * rawSpeedY) + ((1.0f - alpha) * smoothedMoveY);

    // 5. Cộng phần dư còn sót từ lần trước vào giá trị hiện tại
    float totalX = smoothedMoveX + residualX;
    float totalY = smoothedMoveY + residualY;

    // Ép kiểu sang số nguyên để gửi cho máy tính
    int moveX = (int)totalX;
    int moveY = (int)totalY;

    // Lưu lại phần lẻ bị cắt để dùng cho lần kế tiếp (không bị mất chuyển động chậm)
    residualX = totalX - moveX;
    residualY = totalY - moveY;

    // 6. CỘNG DỒN vào bộ đệm chờ gửi, KHÔNG gọi bleMouse.move() ngay.
    // (fix bug khựng chuột: xem giải thích ở khai báo SEND_INTERVAL_MS phía trên)
    pendingX += moveX;
    pendingY += moveY;

    // 7. Chỉ thực sự gửi report BLE theo chu kỳ SEND_INTERVAL_MS cố định
    unsigned long now = millis();
    if (now - lastSendTime >= SEND_INTERVAL_MS) {
        lastSendTime = now;
        if (pendingX != 0 || pendingY != 0) {
            bleMouse.move(clampToInt8(pendingX), clampToInt8(pendingY));
            pendingX = 0;
            pendingY = 0;
        }
    }
}
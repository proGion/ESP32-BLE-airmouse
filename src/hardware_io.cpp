#include "hardware_io.h"
#include <BleMouse.h>
#include "ble_mouse_action.h"

extern BleMouse bleMouse;

// Các chân chuẩn trên mạch KiCad của bạn
#define BATTERY_PIN 32
#define BTN_RIGHT  4
#define BTN_LEFT   5
#define BTN_BACK   18
#define LED_PIN    2
#define LDR_PIN    34

// Thời gian debounce (ms). 15-25ms là đủ cho nút cơ khí thông thường.
// Để 50ms như bản cũ là khá cao, dễ làm nút Trái/Phải cảm giác "lag" khi giữ nhanh.
const unsigned long DEBOUNCE_MS = 20;

// --- CẤU TRÚC TRẠNG THÁI CHO MỖI NÚT ---
// Áp dụng thuật toán debounce kinh điển: chỉ chấp nhận trạng thái mới
// SAU KHI nó đã ổn định liên tục trong DEBOUNCE_MS, không phụ thuộc
// vào việc đó là cạnh lên hay cạnh xuống. Điều này giải quyết triệt để
// cả lỗi "liệt nút" (đọc bỏ lỡ do thời điểm poll không khớp) lẫn lỗi
// "dội phím" (đọc trúng đúng lúc tiếp điểm rung).
struct ButtonState {
    uint8_t pin;
    bool stableState;       // Trạng thái đã được xác nhận ổn định (logic, đã đảo do PULLUP)
    bool lastRawState;      // Trạng thái đọc thô ở lần poll trước
    unsigned long lastChangeTime; // Thời điểm raw state đổi gần nhất
};

ButtonState btnLeft  = { BTN_LEFT,  false, false, 0 };
ButtonState btnRight = { BTN_RIGHT, false, false, 0 };
ButtonState btnBack  = { BTN_BACK,  false, false, 0 };

unsigned long blinkStartTime = 0;
bool isBlinking = false;

void initHardware() {
    pinMode(BTN_LEFT, INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(BTN_BACK, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Khởi tạo trạng thái ban đầu đúng theo mức điện áp thực tế của chân,
    // tránh việc cả 3 nút bị coi là "vừa nhấn" ngay tại thời điểm boot.
    btnLeft.lastRawState  = (digitalRead(BTN_LEFT)  == LOW);
    btnRight.lastRawState = (digitalRead(BTN_RIGHT) == LOW);
    btnBack.lastRawState  = (digitalRead(BTN_BACK)  == LOW);
    btnLeft.stableState  = btnLeft.lastRawState;
    btnRight.stableState = btnRight.lastRawState;
    btnBack.stableState  = btnBack.lastRawState;
}

void setLedState(bool state) { digitalWrite(LED_PIN, state ? HIGH : LOW); }

bool checkSleepMode() {
    if (analogRead(LDR_PIN) < 500) {
        setLedState(true);
        return true;
    }
    return false;
}

// --- HÀM DEBOUNCE DÙNG CHUNG (non-blocking, dựa trên millis()) ---
// Trả về true nếu trạng thái ỔN ĐỊNH của nút vừa đổi trong lần gọi này.
// stableState (sau khi gọi) phản ánh đúng trạng thái thật, đã lọc nhiễu.
bool updateButtonDebounce(ButtonState &btn) {
    bool rawState = (digitalRead(btn.pin) == LOW); // true = đang nhấn (PULLUP nên LOW = nhấn)

    if (rawState != btn.lastRawState) {
        // Tín hiệu thô vừa thay đổi -> reset lại đồng hồ chờ ổn định
        btn.lastChangeTime = millis();
        btn.lastRawState = rawState;

        // Đây là đúng thời điểm ngón tay vừa chạm/rời nút -> dễ làm tay rung
        // nhẹ nhất. Đóng băng chuyển động chuột 1 khoảng ngắn để không ghi
        // nhận rung tay do bấm nút thành chuyển động thật (xem ble_mouse_action.cpp).
        suppressMovementBriefly();
    }

    bool changed = false;
    if ((millis() - btn.lastChangeTime) >= DEBOUNCE_MS) {
        // Tín hiệu đã đứng yên đủ lâu -> chấp nhận là trạng thái thật
        if (btn.stableState != rawState) {
            btn.stableState = rawState;
            changed = true;
        }
    }
    return changed;
}

// Xử lý nút Trái/Phải kiểu nhấn-giữ (drag & drop): gọi press() đúng 1 lần
// tại cạnh xuống, release() đúng 1 lần tại cạnh lên. Dùng chung cho cả 2 nút
// để đảm bảo logic đối xứng, dễ kiểm tra, không bị thiếu nhánh như bản nháp trước.
void handleHoldButton(ButtonState &btn, uint8_t mouseButton, const char *pressMsg, const char *releaseMsg, bool &isHoldingOut) {
    bool justChanged = updateButtonDebounce(btn);
    if (justChanged) {
        if (btn.stableState) {
            Serial.println(pressMsg);
            bleMouse.press(mouseButton);
        } else {
            Serial.println(releaseMsg);
            bleMouse.release(mouseButton);
        }
    }
    if (btn.stableState) isHoldingOut = true;
}

void handleButtons() {
    static bool wasConnected = false;
    bool nowConnected = bleMouse.isConnected();

    if (!nowConnected) {
        wasConnected = false;
        return;
    }

    if (!wasConnected) {
        // Vừa kết nối lại sau khi mất kết nối: đồng bộ lại trạng thái nút
        // theo mức điện áp thực tế, không gọi press()/release() cho trạng thái
        // "ảo" còn sót lại từ trước -> tránh lệch trạng thái với host.
        wasConnected = true;
        bool rawLeft  = (digitalRead(btnLeft.pin)  == LOW);
        bool rawRight = (digitalRead(btnRight.pin) == LOW);
        bool rawBack  = (digitalRead(btnBack.pin)  == LOW);
        btnLeft.stableState  = btnLeft.lastRawState  = rawLeft;
        btnRight.stableState = btnRight.lastRawState = rawRight;
        btnBack.stableState  = btnBack.lastRawState  = rawBack;
        btnLeft.lastChangeTime = btnRight.lastChangeTime = btnBack.lastChangeTime = millis();
    }

    bool isHolding = false;

    // --- 1. CHUỘT TRÁI (Nhấn giữ để kéo thả - Drag & Drop) ---
    handleHoldButton(btnLeft, MOUSE_LEFT, "=> GIỮ CHUỘT TRÁI", "=> NHẢ CHUỘT TRÁI", isHolding);

    // --- 2. CHUỘT PHẢI (Nhấn giữ để kéo thả - Drag & Drop) ---
    handleHoldButton(btnRight, MOUSE_RIGHT, "=> GIỮ CHUỘT PHẢI", "=> NHẢ CHUỘT PHẢI", isHolding);

    // --- 3. NÚT BACK (chỉ click 1 lần khi NHẢ tay, không lặp lại) ---
    bool backJustChanged = updateButtonDebounce(btnBack);
    if (backJustChanged && !btnBack.stableState) {
        // stableState vừa chuyển từ true -> false nghĩa là NHẢ TAY
        Serial.println("=> CLICK BACK");
        bleMouse.click(MOUSE_BACK);

        isBlinking = true;
        blinkStartTime = millis();
        setLedState(true);
    }

    // --- HỆ THỐNG ĐÈN LED ---
    if (isBlinking) {
        if (millis() - blinkStartTime > 100) {
            isBlinking = false;
            if (!isHolding) setLedState(false);
        }
    } else {
        setLedState(isHolding);
    }
}

void updateBatteryLevel() {
    static unsigned long lastBatteryCheck = 0;
    if (millis() - lastBatteryCheck > 60000) {
        lastBatteryCheck = millis();
        if (bleMouse.isConnected()) {
            int adcValue = analogRead(BATTERY_PIN);
            float voltage = (adcValue / 4095.0) * 3.3 * 2;
            int batteryPercent = 0;
            if (voltage >= 4.2) batteryPercent = 100;
            else if (voltage <= 3.2) batteryPercent = 0;
            else batteryPercent = (int)((voltage - 3.2) / (4.2 - 3.2) * 100);

            bleMouse.setBatteryLevel(constrain(batteryPercent, 0, 100));
        }
    }
}

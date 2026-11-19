#ifndef BLE_MOUSE_ACTION_H
#define BLE_MOUSE_ACTION_H

void initBleMouse();
// Đổi tên biến thành angleX và angleY cho tổng quát
void processMovement(float angleX, float angleY);

// Gọi hàm này ngay tại thời điểm phát hiện nút vừa được chạm/nhả (raw state
// đổi), để tạm "đóng băng" chuyển động chuột trong 1 khoảng ngắn. Mục đích:
// khi ngón tay bấm/nhả nút, cả bàn tay thường rung/nghiêng nhẹ -> gyro ghi
// nhận nhầm thành chuyển động thật, khiến chuột nhảy đúng lúc định click.
void suppressMovementBriefly();

#endif
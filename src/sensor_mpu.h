#ifndef SENSOR_MPU_H
#define SENSOR_MPU_H

void initMPU();
bool updateMPU(); // trả về true khi vừa có mẫu cảm biến mới (đúng chu kỳ 400Hz)
float getPitch();
float getYaw();
float getRoll();
#endif
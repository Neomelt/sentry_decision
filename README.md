小陀螺：需要和电控沟通多写几种转速还是完全交给上位机

发点：从yaml读航点，但是感觉可以写成条件发点

全向感知


需要的裁判系统数据：基地前哨的血量，自身的血量，弹丸数量，当前姿态


必须加帧头帧尾，校验位(CRC16)
TX_nav(13+3)：
``` cpp
struct{
    float vx;
    float vy;
    float wz;
    uint8_t attitude;  # 按照规则约定编号
}
```

TX_aim(12+3):
``` cpp
struct{
    float gimbal_target_roll;
    float gimbal_target_pitch;
    float gimbal_target_yaw;
}
```

RX_all(31+3):
``` cpp
struct{
    uint8_t remaining_health;
    uint8_t bullet_num;
    uint8_t red_outpost_health;
    uint8_t blue_outpost_health;
    uint8_t red_base_health;
    uint8_t blue_base_health;
    uint8_t aim_mode;
    float chassis_roll;
    float chassis_pitch;
    float chassis_yaw;
    float gimbal_roll;
    float gimbal_pitch;
    float gimbal_yaw;
}

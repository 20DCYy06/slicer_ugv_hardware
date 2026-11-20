# v1.3 1m/s速度上限候选（未刷写）
只将max_body_mm_s与max_wheel_mm_s从800改为1000；PID/加速度/IMU/PB15/故障逻辑/80%PWM均不变。
配套上位机待激活文件在pending_host/：combined_hardware.yaml与combined_gamepad.json。
刷写通过后，将两文件复制到Test/tests/，融合入口即可使用专用0.1–1.0m/s档位（0.05步进，共19档）。
不会修改Hardware/config.yaml，原WASD、独立手柄配置不变；角速度0.05–0.5rad/s不变。
当前已安装v1.2会拒收>0.8m/s，未刷写前不能激活新档位配置。
编译日志Hardware/Logs/build_v1.3_speed.log，未运行实物，不保证80%PWM下机械速度必然达到1m/s。

# v1.2 PB15扩展（已安装验收）

从本机当前v1.1/src复制，基线SHA256清单baseline_v1.1_sha256.json。
只扩展PB15 TIM12_CH2及0x30/0x31命令，车轮、IMU、PID和现有协议保持原实现。
编译命令：`cd /home/potato/Slicer_Workspace2 && ./Hardware/build.sh v1.2`。
二进制：bin/firmware.bin；测试及编译日志：../../Logs/build_v1.2_pb15.log。
上位机：./Test/run_b15_servo.sh --run（路径请从工作空间根目录执行）。

新增PB15能力查询、PWM/CRC/范围检查、500ms失联中位测试通过，既有车轮/IMU测试通过。
已刷写；整片512KiB与选项字节校验、四路零速验收及PB15能力查询通过。未实物运动验证。
刷前重新确认12V断开、Motor OFF、仅USB并备份现板，禁止默认覆盖他人更新。

# v1.1 轮端坐标与IMU符号一致性修复

依据 Test/logs/feedback_20261107T124431_528345Z.jsonl：向车头推车时四轮带符号距离全负；手动左转+57.9°、右转-63.28°；旧轮位推导yaw和IMU相关系数-0.9338。A/C旧轮速与IMU负相关，B/D正相关。

修复对比：

| 参数 | v1.0 | v1.1 |
|---|---|---|
|encoder_signs|[-1,1,-1,1]|[1,-1,1,-1]|
|pwm_signs|[-1,1,1,1]|[1,-1,-1,-1]|
|wheel_side|[1,-1,1,-1]|保持|
|IMU gyro_signs|[1,1,1]|保持|

成对反转编码器与电机输出符号，保持轮速内环的执行/反馈方向一致；轮速正向恢复为车头前进，正yaw对应IMU的逆时针。PID、PI、硬件引脚及算法主体不改，不新增滤波、死区或状态机。轮径65mm、PWM上限80%、原保护配置不变。

仅重编译固件默认值不够：上位机启动会用0x12重发极性配置。因此配套host_bundle必须在烧录验证成功后同步。WASD恢复W正vx/A正wz，手柄恢复轴取负；用户实际前后左右操作保持正常，不再用反向上位机指令掩盖底层坐标反号。

validation/feedback_validation.json包含真实手推数据重算与实际balance.c的离线响应模型。模型假设轮速内环可跟踪，不能替代电机实际正向、制动或地面跟踪验收。

编译：在Workspace2运行 ./Hardware/build.sh v1.1。
烧录：确认12V物理断开、Motor OFF，仅USB供电后，运行 python3 -s Hardware/Flashing/flash.py --version v1.1 --flash --power-off-confirmed。
烧录与整片校验通过后才部署host_bundle并执行Hardware/zero_check.py。所有备份和日志保留在Workspace2，v1.0及旧工作空间保持原样。

原测试曾把旧车头方向的开环符号写死。v1.1测试改为验证按当前pwm_signs配置后，与等效桥臂开环电气指令的8路CCR一致；没有删除桥臂校验。核心C源码未改变。

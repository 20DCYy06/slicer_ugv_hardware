# Workspace2 独立底盘测试

WASD默认低档：线速度0.10 m/s，角速度0.1 rad/s。R升档、E降档，分别按0.05 m/s与0.1 rad/s递增/递减，独立到各自上限0.60 m/s和1.0 rad/s后饱和；两轴上下限所需档数不同，不强制按同一整数档映射。

## WASD（默认SSH终端）

```bash
cd /home/potato/Slicer_Workspace2
./Test/run_wasd.sh --demo   # 只显示R/E档位，不连接底盘
./Test/run_wasd.sh --check  # 只读底盘状态
./Test/run_wasd.sh          # 在当前SSH终端直接按键，不用回车
./Test/run_wasd.sh --gui    # 可选图形模式，需点击窗口获得焦点
```

默认终端模式不依赖X11；长按W/S前后、A/D转向，松开后最多0.6秒无方向键重复便令目标平滑归零，空格立即令目标归零。此为终端无松键事件的兼容方式；可在Hardware/config.yaml的keyboard.repeat_timeout_s调整。图形模式支持W+A、W+D组合圆弧及真实松键事件；松开、失焦或空格令目标归零，按共享加速度平滑减速。Q/Esc或关闭窗口结束并立即发全停止。图形模式R/E首次按下调一档，终端模式每次收到R/E字符调一档。优先使用当前DISPLAY和认证文件：MobaXterm的localhost:10.0转发不会再混入GDM认证。未设置DISPLAY时自动使用Jetson本地:0及其GDM认证。若想在RustDesk连接的Jetson桌面显示，使用`./Test/run_wasd.sh --local-display`。显示连接成功后才打开底盘串口。

## 双摇杆手柄

```bash
cd /home/potato/Slicer_Workspace2
./Test/run_gamepad.sh --demo
./Test/run_gamepad.sh --check
./Test/run_gamepad.sh
```

左摇杆Y（轴1）给前后油门，右摇杆X（轴2）控制左右转向；两个轴的下发方向已相对初版同时反转，符号在gamepad.throttle_sign、steering_sign配置。手柄角速度档位单独为0.05～0.5 rad/s，每步0.05，线速度仍为0.1～0.6 m/s；WASD角速度仍按原档位。左右各给独立分量，可同时使用。当前USB WirelessGamepad的Linux轴代码[0,1,2,5,16,17]与旧程序对应；不使用左X替代右X。

当前手柄RB/TR对应按钮7升档，LB/TL对应按钮6降档；终端R/E也可调档。B按钮1或空格停止，终端Q/Esc退出。轴回中时平滑归零；停止键后需重新拨杆才恢复。拔出手柄或程序异常会发送全停止；主机断开后MCU原500ms看门狗兜底。其他手柄可在`Hardware/config.yaml`调整device、轴/按键编号与死区。

## 速度与共用链路

两程序只发送0x16的线速度/角速度，绝不发送0x11 PWM；编码器及IMU闭环留在STM32。共享`control_core.py`和`Hardware/Host/chassis.py`；50Hz刷新、默认线加速度0.5m/s²、角加速度1.5rad/s²。最高档是速度上限，手柄可在范围内连续给0~100%输入，零输入可停机，不把0.1最低档误当作永远不能低于0.1的实际速度。

两程序不能同时占用同一串口；按USB序列号匹配，所有参数在`Hardware/config.yaml`。协议、硬件参数和解锁见`Hardware/README.md`。运行无需ROS、pygame或旧工作空间；依赖Python3、PyYAML，WASD另需Tkinter（本机均已有）。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s -m unittest discover -s Test/tests -v
```

`--demo`和离线单元测试都不连接电机，不等同于实际行驶验收。

WASD方向校正：仅反转键盘输入到vx/wz的映射，终端与GUI模式同时生效。W下发负vx、S下发正vx；A下发负wz、D下发正wz，以匹配当前车身实测方向。手柄映射和固件不变。

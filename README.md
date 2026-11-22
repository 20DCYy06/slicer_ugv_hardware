# Slicer UGV 硬件

本仓维护 C30D 固件、SLC2 协议、设备配置和底层主机工具。整车仓库通过 `dependencies.repos` 固定与整车版本配套的硬件提交。

- `Firmware/` 按版本保存固件源码、构建清单和发布镜像。
- `Host/` 保存底层主机控制接口。
- `Flashing/`、`Unlock/` 和 `LockDown/` 保存底层设备工具。
- `last_install.json` 标识本版本配套的发布镜像。设备验收和现场回退以各车的安装记录为准。

从本仓根目录执行 `./build.sh v1.7` 可运行离线检查并构建对应固件。开发协作见 [AGENTS.md](AGENTS.md)，镜像和版本说明见 [v1.7](Firmware/v1.7/README.md)。整车使用入口见 [slicer-ugv](https://github.com/20DCYy06/slicer-ugv)。

`Host/rtk/` 提供 RTK 驱动使用的串口、协议和里程计核心。整车构建负责安装这些模块。来源、对应提交及声明见 [RTK 模块来源记录](Host/rtk/SOURCES.json)。

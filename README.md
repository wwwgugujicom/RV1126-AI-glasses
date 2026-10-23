# RV1126B AI Glasses Driver
基于 RV1126B SDK 的 AI 眼镜自定义驱动代码（SC235HGS/SC533HGS/OC0TB1B/SC132HGS 摄像头 + ICM426x IMU + 4cam dtsi）。
基于 RV1126B SDK 的 AI 眼镜自定义驱动代码。

## 内容     
- drivers/media/i2c/sc235hgs.c —— SC235HGS 摄像头传感器驱动
- drivers/media/i2c/sc533hgs.c —— SC533HGS 摄像头传感器驱动
- drivers/media/i2c/oc0tb1b.c  —— OC0TB1B  摄像头传感器驱动
- drivers/media/i2c/sc132hgs.c —— SC132HGS 摄像头传感器驱动
- drivers/io/imu/icm4268x.c —— ICM-426x IMU 驱动
- rv1126b-4cam.dtsi —— 4 摄像头设备树配置

## 使用方法
1. 获取对应版本的 RV1126B SDK（Rockchip 原厂/模组厂提供）
2. 将本仓库文件按相同路径放入 SDK 对应位置，
   并在相应 Makefile / Kconfig 中添加编译项
3. 按 SDK 正常流程编译内核并烧录

# 视觉识别程序

本目录收录原工程 `gcs-gold-medal-main/cv2_python/zongchengxu20250724chusai.py`，文件名保留原版本日期。代码用于二维码、颜色及机构定位等视觉任务，并通过串口与 STM32 通信。本次只复制源文件，未改动识别算法，也未在相机和实车上验证。

## 运行前检查

- 脚本依赖 Python 包 `pyserial`、`opencv-python`、`numpy`、`pyzbar`；`pyzbar` 还依赖系统中的 ZBar 库。
- 代码在加载时就会打开 `/dev/ttyUSB0`，波特率为 9600、8N1。请先确认它指向连接 STM32 的串口。
- 代码使用 `/dev/video_xia0` 和 `/dev/video_shang0` 两个摄像头设备路径；请先核对设备映射和实际镜头位置。
- 与 STM32 的收发格式要同时对照本脚本的 `send_data()`、`receive_data()` 和固件 `SYSTEM/Serial.c`、`USER/main.c`。单凭串口波特率相同不能证明数据包兼容。

运行此脚本会尝试打开串口和摄像头，并可能发送指令；先在目标设备上核对硬件连接与任务模式。若要移植到新环境，优先把设备路径和串口参数改为可配置项，再逐项验证识别及通信。

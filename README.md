# FPGA 实时图像畸变矫正系统

本项目面向紫光同创赛题二，实现基于 FPGA 的实时图像畸变矫正，支持普通矫正、IMU 动态矫正、平面扫描和自适应图像增强。

装置由 PGL50H FPGA 开发板、OV5640 鱼眼摄像头、DDR3、IMU、HDMI 显示或采集设备及上位机组成。FPGA 执行坐标映射、映射表生成、像素取样、双线性插值和视频输出；上位机负责标定参数管理、姿态控制、预览、拍照录像及指标展示。

## 下载完整工程

**[打开 C129 最终版 Release，下载完整工程 ZIP](https://github.com/Yuehuan111/c129-fpga-vision-final-20261104/releases/tag/C129-final-20261104)**

请在 Release 页的 **Assets** 中下载 **`C129_final_code_and_all_data.zip`**。完整工程源码、PDS 工程、时序与引脚约束、配套 IP、位流、上位机、C97 标定参数、交付包内的全部运行日志、原始测试图片、画质报告及复核程序均在这个附件中。

本仓库 **Code 页面已展开全部工程与数据**，入口为 [`C129_compiled_final_20261031/`](C129_compiled_final_20261031/)。可直接浏览源码，或通过 **Code → Download ZIP** 下载当前分支。4 个超过 GitHub 单文件限制的历史日志以无损 `.jsonl.gz` 保存，使用前可解压，详情见 [Code 文件说明](docs/Code文件说明.md)。Release 页的自动 `Source code (zip/tar.gz)` 对应发布时的说明快照；原始完整包请下载上方具名附件。

Release 附件原文件名为 `最终版本_代码与全部数据文件.zip`，仅为便于下载改为英文名，文件内容不变。完整 ZIP 的 SHA256 为：

```text
fc967f2c99bf21b17e80ee0780e2514d2b15069b7e1e3d56da943d38bb067167
```

核对文件：[原 ZIP 的 SHA256 校验值](SHA256SUMS.txt) · [全部 547 个文件的路径、大小与 SHA256 清单](package_manifest.json)。`SHA256SUMS.txt` 和清单保留原文件名，校验时以 SHA256 值匹配下载附件。

## 使用步骤

1. 下载 Release 完整 ZIP 时，解压后进入 `C129_当前版本_代码与全部依据_20261103_213455/C129_compiled_final_20261031/`；下载 Code 当前分支时，进入仓库内的 `C129_compiled_final_20261031/`。
2. 连接摄像头、IMU、HDMI 和串口，下载包内 `C129_720p_cmos1.sbit`。
3. 首次使用按工程内 `使用说明.txt` 配置 Python 和上位机依赖，然后运行 `start_demo.cmd`。
4. 在上位机选择实际设备并加载标定参数，即可切换普通矫正、IMU 动态矫正、平面扫描和自适应图像增强等演示模式。
5. 动态矫正时先记录参考姿态，再同步转动相机与 IMU，保持两者相对安装关系不变。
6. 按 **F8** 拍照；按 **F9** 开始或停止录像。

已有 PDS 工程入口为 `fpga_reference/project/board/build_cmos1/camera_board50.pds`。完整运行、依赖安装及编译说明以 ZIP 内 `使用说明.txt` 为准。

## 已有测试指标

| 指标 | 数值 |
| --- | --- |
| 视频处理分辨率 | 1280×720 |
| 板端日志记录的新矫正帧率 | 30 fps |
| HDMI 刷新率 | 约 60 Hz |
| 原图整体直线度 RMS | 2.201 px |
| 矫正后整体直线度 RMS | 0.282 px，降低 87.17% |
| 原图近似 MTF50 | 0.3468 cycles/pixel |
| 矫正后近似 MTF50 | 0.3269 cycles/pixel |
| 普通矫正首像素延迟 P50 / P95 | 71.902 / 71.955 ms |
| 普通矫正整帧输出完成延迟 P50 / P95 | 87.899 / 87.952 ms |
| LUT | 30263 / 42800，70.71% |
| FF | 34299 / 64200，53.43% |
| APM | 79.5 / 84，94.64% |
| DRM | 92 / 134，68.66% |
| PDS 慢角 Setup / Hold 裕量 | +0.203 / +0.051 ns |

帧率与延迟来自板端运行日志。新矫正帧率与 HDMI 刷新率分别统计；30 fps 来自 FPGA 帧计数，不以电脑预览或录像文件的帧率替代。延迟表采用普通矫正稳定段的 69 个有效匹配样本，按 FPGA 图像输入首像素至对应帧输出首像素、末像素分别计算，不含曝光、采集卡、USB、电脑预览及显示面板延迟。

画质数据来自网格及斜边图卡测试。整体 RMS 指该批提取线点的合并统计；近似 MTF50 使用两图共同有效的局部斜边，按原始像素计量。测试方法、样本范围、计数窗口口径及数据出处见 **[性能分析报告](docs/性能分析报告.txt)**。

## 文件与证据索引

| 内容 | 位置 |
| --- | --- |
| 可直接浏览的最终工程源码、位流与数据 | [C129_compiled_final_20261031/](C129_compiled_final_20261031/) |
| 原始指标交付资料 | [原始指标资料/](原始指标资料/) |
| Code 文件完整性清单及大日志说明 | [code_import_manifest.json](code_import_manifest.json) · [Code文件说明](docs/Code文件说明.md) |
| 完整工程、位流、上位机与全部交付数据 | [Release 完整 ZIP](https://github.com/Yuehuan111/c129-fpga-vision-final-20261104/releases/tag/C129-final-20261104) |
| 性能指标、定义、测试范围与包内证据路径 | [docs/性能分析报告.txt](docs/性能分析报告.txt) |
| 原 ZIP 的 SHA256 | [SHA256SUMS.txt](SHA256SUMS.txt) |
| 原 ZIP 内每个文件的路径、大小与 SHA256 | [package_manifest.json](package_manifest.json) |


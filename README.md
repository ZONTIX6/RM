# RM

本仓库面向 RoboMaster 赛事开发，整理无人机/机械臂 MuJoCo 仿真和 STM32 电控开发资料。

| 目录 | 内容 | 入口 |
| --- | --- | --- |
| [aircraft](aircraft/README.md) | Phantom 4 Pro 单机、中文注释教学版、三机 DMPC + CBF + SE(3) 编队仿真 | `simulate.py`、`swarm_sim.py` |
| [basic_framework/mujoco_arm](basic_framework/mujoco_arm/README.md) | 六自由度机械臂、目标运动、重复运动和抓取演示 | `run_ntu_arm.py` |
| [basic_framework](basic_framework/README.md) | STM32F407 电控框架、驱动、机械臂运动学/轨迹规划/重力补偿、差分腕和堵转检测 | `CMakeLists.txt`、`application/`、`modules/` |
| [CAD 工具](basic_framework/tools/README.md) | SolidWorks 导出 STL、Blender 合并 GLB 和渲染预览 | `basic_framework/tools/` |
| [开发文档](basic_framework/.Doc) | 原框架架构、调试、PID 和 CAD 结构参考资料 | Markdown / PDF |

## 仿真快速开始

推荐 Python 3.12。以下命令从仓库根目录执行，先安装所选仿真的依赖。

```sh
cd aircraft
python -m pip install -r requirements.txt
python simulate.py                        # 无头仿真，输出轨迹和误差
python simulate.py --render               # 单机窗口
python swarm_sim.py --seed 7               # 可复现的三机编队
python swarm_sim.py --render --seed 7      # 三机窗口
```

机械臂：

```sh
cd basic_framework/mujoco_arm
python -m pip install -r requirements.txt
python run_ntu_arm.py --pick-demo
python run_ntu_arm.py --repeat-target --print-joints
```

Windows 可在对应目录运行 `py -3.12 -m venv .venv`，再用
`.venv\Scripts\python.exe -m pip install -r requirements.txt` 安装依赖。
随后双击该目录的 `.bat` 启动器；它们优先使用同目录 `.venv`，其次查找系统 Python。
Linux/macOS 可用 `python3 -m venv .venv` 创建环境并激活后执行以上 Python 命令。

## 电控框架

固件工程位于 `basic_framework/`，包含 STM32 HAL、CMSIS/DSP、FreeRTOS 和原有板级文件，
构建配置及开发说明见[原框架 README](basic_framework/README.md)。
机械臂相关源码位于 `application/roboticarm/`、`modules/algorithm/` 和
`modules/motor/diff_wrist/`。CMake 的 `ROBOTIC_ARM_ENABLED` 默认关闭，可按需开启。
MuJoCo 机械臂使用 Python 脚本轨迹和位置伺服，与固件控制是独立入口。

仿真通过无头运行检查；固件源码当前有原始语法错误和未使用变量导致的构建失败，
尚未达到可直接烧录的状态。具体结果和测试边界见[发布前验证](VALIDATION.md)。

## 来源和整理范围

电控框架源自 [HNUYueLuRM/basic_framework](https://github.com/HNUYueLuRM/basic_framework)，
保留其 [MIT 许可](basic_framework/LICENSE)及各文件的原版权声明；第三方驱动、中间件和
机械臂模型中的原作者声明保持原样。该 MIT 许可不用于替代其他目录或第三方文件原有的许可。

本次整理保留仿真源码、模型、教程、展示图、固件工程和可复用的 CAD 转换工具。
不包含本地虚拟环境、缓存、编译输出、个人 IDE/助手配置、仿真输出轨迹，
以及大体积 SolidWorks/STEP/STL/GLB 和视频资料。部分原始开发文档会引用本次未收录的 CAD 或视频。
原 CAD 网页预览依赖未包含的 API 服务和 Three.js 资源，因此未作为独立项目收录。

仿真展示用于控制算法与运动学学习；模型参数推导和脚本抓取流程的实现细节见各目录说明。

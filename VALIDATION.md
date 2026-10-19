# 发布前验证

日期：2026-10-19。测试环境：Windows、Python 3.12.14、MuJoCo 3.12.0、
NumPy 2.3.5、Matplotlib 3.11.1。

## 无人机

- `python simulate.py` 和 `python simulate_annotated.py`：各完成 16 s 无头仿真，
  RMS 跟踪误差 2.74 cm，最大误差 5.55 cm。
- `python swarm_sim.py --seed 7`：完成 14 s 无头仿真，最终目标误差打印值为
  0.00 cm，最终编队误差 0.02 cm，10 cm 阈值下收敛时间 9.34 s，
  最小机间距 0.79 m。该次运行 CBF 修正次数为 0，结果只对应此初始条件，
  不构成对所有初始状态避碰能力的验证。
- 单机/集群生成的轨迹数组均为有限数值，原始和生成的 MJCF 模型均能加载。

## 机械臂

- `robticarm.xml` 能加载，6 个位置执行器正常创建。
- 无界面执行一个完整的 14 s 抓取周期，检查有限关节状态、抓取前物块位置、
  抓取后跟随位置、释放后放置位置及指定目标姿态，均通过。
- 抓取演示通过脚本轨迹和 mocap 物块跟随实现，以上检查不验证接触力学抓取。
- 本次没有打开 GUI 手动检查交互窗口。

## 固件和 CAD 工具

使用 CMake + Ninja 和 Arm GNU 14.3.1 配置固件工程成功，默认配置编译失败：

- `basic_framework/Src/main.c:79` 的函数开头带有原始源码中的多余 `+` 字符（`+{`），
  造成语法错误。
- `basic_framework/application/chassis/chassis.c:49` 起的 `buffer_PID`、
  `referee_data`、`ui_data`、`cap` 对应使用代码已注释，在 `-Werror` 下成为未使用变量错误。

本次按源码资料收录固件，保留上述原始问题；未验证机械臂启用配置、板上运行和硬件联调。
使用前需先修复编译问题并按自己的板卡配置参数。

发布目录内所有 Python 源文件通过语法解析。CAD 转换/渲染脚本未运行
SolidWorks 或 Blender，仅进行了 Python 语法检查。

已排除虚拟环境、助手配置、缓存、固件构建输出和仿真运行数据。

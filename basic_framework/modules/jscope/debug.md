# debug module

`modules/debug` 用来放调试辅助代码，避免把 RTT、JScope、CSV 打印等代码直接写在应用层业务文件里。

## RTTDebugJScope2F

`RTTDebugJScope2F` 发送一种固定格式的数据包：

```c
uint32_t time_us;
float value0;
float value1;
```

当前底盘用它发送单个 M3508 的目标速度和反馈速度：

- `value0`：目标速度 `ref`
- `value1`：反馈速度 `fb`
- RTT 通道：默认通道 1
- JScope buffer name：`JScope_t4f4f4`

## 底盘波形选择

在 `application/chassis/chassis.c` 里改这些宏即可：

```c
#define CHASSIS_JSCOPE_CH 1
#define CHASSIS_JSCOPE_WHEEL CHASSIS_JSCOPE_WHEEL_LF
#define CHASSIS_RTT_CSV_LOG_ENABLE 0
#define CHASSIS_RTT_CSV_LOG_DIV 1
```

轮子选择：

```c
CHASSIS_JSCOPE_WHEEL_LF
CHASSIS_JSCOPE_WHEEL_RF
CHASSIS_JSCOPE_WHEEL_LB
CHASSIS_JSCOPE_WHEEL_RB
```

如果只用 JScope 看实时曲线，保持 `CHASSIS_RTT_CSV_LOG_ENABLE` 为 0。需要在 RTT Viewer 里记录数值时，把它改成 1，并用 `CHASSIS_RTT_CSV_LOG_DIV` 控制打印分频。
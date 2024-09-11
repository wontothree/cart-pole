# Firmware

# Functions

|Function Name|Description|
|---|---|
|init_UART||
|moveOneStep|apply voltage to each coils A, B, A_, B_|
|setup||
|loop||

# Variables

|Type|Variable Name|Initialization|Description|
|---|---|---|---|
||step_info[8][4]||스텝 모터의 각 코일에 인가할 전압 패턴을 정의한다. HIGH는 해당 코일에 전압이 인가됨을, LOW는 전압이 인가되지 않음을 나타낸다. 스텝을 변경할 때마다 다른 코일에 전압이 인가되어 모터가 회전한다.|
|float|vel_current|0|current velocity|
|float|vel_target|2|target velocity|
|uint16_t|interval|500|Time between steps in timer ticks|
|uint16_t|time_last_step|0||
|uint16_t|time_last_ctrl|0||
|uint16_t|time_current||current time|
|||||

# Dependencies

```py
pip install pyserial
```

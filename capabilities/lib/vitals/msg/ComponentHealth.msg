uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
string name                  # 部件名, 如 "cpu", "gpu", "battery"
uint8 health                 # OK / WARN / ERROR
string detail                # 人读详情, 如 "CPU temp 85C exceeds 80C threshold"
float32 value                # 当前值 (温度:℃, 电压:V, 电流:A), -1 无
float32 threshold            # 触发阈值, -1 无

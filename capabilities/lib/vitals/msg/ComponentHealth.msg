uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
string name                  # component name, e.g. "cpu", "gpu", "battery"
uint8 health                 # OK / WARN / ERROR
string detail                # human-readable detail, e.g. "CPU temp 85C exceeds 80C threshold"
float32 value                # current value (temp:°C, voltage:V, current:A), -1 if absent
float32 threshold            # threshold that was crossed, -1 if absent

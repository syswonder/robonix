uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 STALE=3
uint8 UNKNOWN=4
string name                  # component name, e.g. "cpu", "gpu", "battery"
uint8 health                 # OK / WARN / ERROR / STALE / UNKNOWN
string detail                # human-readable detail, e.g. "CPU temp 85C exceeds 80C threshold"
float32 value                # current value (temp:°C, voltage:V, current:A), -1 if absent
float32 threshold            # threshold that was crossed, -1 if absent

# Current device-reported contact state, not the result of a specific command.
uint8 UNKNOWN=0          # No trustworthy contact observation.
uint8 NOT_HOLDING=1      # Device confirms no established grip.
uint8 HOLDING=2          # Contact established; no achieved-force guarantee.
uint8 SLIPPING=3         # Established contact is slipping according to the device.

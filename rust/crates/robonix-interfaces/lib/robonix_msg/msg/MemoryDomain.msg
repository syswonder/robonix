# Memory domain where a Robonix buffer resides.
#
# Determines how cross-process sharing and GPU access are handled.
# On Jetson (UMA), CPU and GPU share physical DRAM so UNIFIED is
# the default; on discrete GPU systems, buffers start as CPU and
# are pinned for GPU DMA as needed.

uint8 CPU     = 0
uint8 GPU     = 1
uint8 UNIFIED = 2

uint8 domain

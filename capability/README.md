# Capability

Standard capability units callable by the intelligence layer (e.g., via MCP or `brain`). Each capability wraps an isolated function and should include the following structure:

## ✅ Typical Capabilities

- `audio_speak`: Text-to-Speech
- `mic_listen`: Audio input/command
- `nav2_walk`: Navigation command interface
- `yolo_look`: Visual detection interface
- `grasper`: Robotic arm control
- `internet`: External info retrieval

## 📁 Directory Structure

capability/
└── nav2_walk/
├── api/ # Provides: init(), start(), soft_config(), act(), sense(), emergency(), standby(), shutdown()
├── description/ # Framework configuration, e.g., auto-start, parameters
├── src/ # Implementation logic
└── README.md # Developer manual
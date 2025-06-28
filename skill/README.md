# Skill

Skill modules represent high-level actions built from multiple capabilities. These can be implemented as event flows, finite-state machines, or lightweight agent behaviors.

## 🧠 Examples

- `hello_spin_greet`: Rotate → Detect human → Speak greeting
- `pick_cup`: Visual locate → Navigate → Grasp object
- `make_bed`: Multi-step sequence with planning and feedback

## 🔁 Lifecycle

Skills are invoked by the Brain and may be re-entrant, interruptible, and stateful. Skills may also self-adapt based on task history via Memory or Cost inputs.

# What is the application?

2026-09-20. Written before adding anything else to Scene, because the last
few weeks of work on it have been driven by defects rather than by a claim
about what somebody is going to ask the robot to do.

The order here is deliberate: what we can observe about real use, what the
hardware allows, what prior work measured about how people actually phrase
requests, and only then what Scene should store and expose. Everything with
a number attached has a source.

---

## 1. What one real session actually looked like

Logs from the deployed robot, 2026-09-20 17:32–17:46, 14 minutes.

```
goals received            14, every one of them "你好"
plans produced            10 x "no new work this round"
                           5 x "播放你好语音" / "say hello through the speaker"
                           3 x "wait"
```

Two things are visible in that.

**The wake phrase became the task.** The configured wake word is `你好`;
every `task_update` in the session carries `goal='你好'`. Liaison hands the
utterance that woke the robot to Pilot as the request, so the robot spent
the session being asked to say hello.

**The first real question had no answer path.** One goal reads
`'你好你有哪些技能你好你有哪些技能你好，你有哪些技能？'` — the ASR
concatenation of somebody asking *"what skills do you have?"* three times.
Each attempt produced `no new work this round`.

A person meeting this robot asks what it can do. That question is not a
navigation query, not a perception query, and not in any contract we have.
It is also the only question we have direct evidence of a real user asking.

---

## 2. What the hardware allows

Both the simulated and the physical deployment:

| | sim (`soma.yaml`, in use) | physical (from provider logs) |
|---|---|---|
| base | mobile_base, 2 wheels | benben_chassis |
| perception | rgbd_camera, lidar_2d | camera, mid360 lidar + imu |
| interaction | audio_io | audio_driver, client bridge |
| manipulation | **none** | **none** |

`soma.full.yaml` adds a 7-DOF arm and gripper, in simulation only.

This rules out most of what the household-robotics literature measures.
BEHAVIOR-1K surveyed 1,461 people over ~2,000 activities and reports that
tedious chores score highest and recreation lowest, with roughly 200
cleaning and 200 cooking activities in the benchmark — nearly all of it
manipulation. None of that is reachable here.

What is reachable is the set of tasks built from **move, look, listen,
speak, remember**. That is the honest envelope, and it is not a small one:
it covers finding things, answering questions about a space, going
somewhere, monitoring, and guiding.

---

## 3. What prior work measured about how people ask

### The task space is small

Walker, Peng and Cakmak (2019) annotated the RoboCup@Home 2018 GPSR command
generator, the standard "arbitrary spoken command" benchmark for
general-purpose service robots:

| | Cat 1 | Cat 2 | Cat 3 | All |
|---|---|---|---|---|
| distinct anonymized commands | 192 | 352 | 667 | **1211** |
| distinct logical forms | 17 | 39 | 45 | **101** |
| commands per form | 11.3 | 9.0 | 14.8 | **12.0** |

The whole space is **27 predicates: 7 actions and 20 descriptive**. 1211
distinct commands collapse to 101 logical forms, and they collected 1,836
crowdsourced paraphrases from 95 workers on top of that without the form
count growing.

The design consequence is direct: the interface Scene needs to support is
not open-ended. It is a couple of dozen predicates, and getting those right
matters more than breadth.

### The categories are about specification quality, not verbs

The three GPSR categories are not "navigation / manipulation / speech". They
are:

1. **fully specified** — "Move to the dinner table, grasp the Energy drink
   and put it in the trash bin."
2. **underspecified** — "Bring me some Drink from a shelf." The drink and
   the shelf are whichever ones exist.
3. **erroneous** — "Bring me the Milk from the dresser", where there is no
   milk. The robot is scored on detecting and reporting that.

Categories 2 and 3 are Scene's problem specifically. Resolving "a shelf" to
a shelf is a query against what the robot has seen; answering "there is no
milk" requires distinguishing *never seen* from *seen and now gone*, which
is exactly what the departures table was built for and what no contract
currently exposes.

Their example logical forms show what the predicates look like in practice:

```
say(count(λ$1.(is_a($1,<object>)) ∧ at($1,<location>)))          "how many cokes are in the freezer"
say(λ$1.(largest($1) ∧ at($1,<location>)))                        "which is the largest object on the bar"
bring(λ$1.(λ$2.(is_a($2,<object>) ∧ on_top_of($1,$2))),<location>) "the thing on the coffee table that's on the glass"
```

### Ambiguity is the normal case, not a failure

ReferIt3D: Nr3D is 41,503 human-written referring utterances, Sr3D is
83,572 template-generated ones. Human listeners resolve Sr3D at 92% and
Nr3D at **86%** — people cannot reliably resolve each other's referring
expressions either.

So a system that asks "which one?" is not degrading. It is doing what the
14% requires. `find`'s discriminating question is the right shape; what is
missing is that nothing currently calls it.

Sr3D's five relation types, by share of the dataset: horizontal 81%,
between 8%, allocentric 5%, vertical 4%, support 2%.

---

## 4. A task taxonomy for *this* robot

Combining the envelope in §2 with the predicate set in §3, and dropping
everything that needs an arm:

| # | Kind | Example utterance | What it needs from Scene |
|---|---|---|---|
| T1 | capability | "你有哪些技能？" | nothing — but nothing answers it today |
| T2 | existence | "有没有看到我的杯子？" | class query + *never seen* vs *gone* |
| T3 | location | "椅子在哪？" | class query + region containment |
| T4 | count | "会议室有几把椅子？" | aggregate over a filtered set |
| T5 | reference | "桌上那个显示器" | class + relation + disambiguation |
| T6 | go to | "去我常坐的那把椅子" | reference + approach pose + **visit history** |
| T7 | guide | "带我去打印机" | reference + navigation + speech |
| T8 | inventory | "这个房间里有什么？" | region query, grouped, deduplicated |
| T9 | monitor | "有人进来告诉我" | standing query over a stream |
| T10 | recall | "我昨天把钥匙放哪了？" | history of an object, not its current state |

T1 is the only one we have evidence of a real user asking. T5/T6 are what
the grounding work has been aimed at. T9 and T10 are outside everything
Scene currently offers, in different ways: T9 needs a subscription, T10
needs a timeline.

---

## 5. Gap against the contracts Scene ships

Scene currently declares 14 contracts:

```
delete_object  find  flush_objects  get_object_context  get_robot_context
get_scene_graph  goal_near  goal_region  go_to  list_objects  list_regions
list_relations  update_object_geometry  update_object_label
```

Relations maintained: `near / on_top_of / under / inside / contains` (LLM,
30s) and `reachable_by` (geometric, 3Hz).

| Need | Covered by | Gap |
|---|---|---|
| `is_a`, class filter | `list_objects` | client-side filtering only |
| `at(obj, region)` | `list_regions` + containment | containment is the caller's to compute |
| `on_top_of`, `under`, `inside` | `get_scene_graph` | — |
| **`left_of` / `right_of` / `in front of`** | — | **absent. 81% + 5% of Sr3D's relations** |
| `count` | — | absent; every caller re-implements it |
| `largest` / superlatives | `find` has closest/farthest | no size, no other attributes |
| `person`, `name` | — | no person entity at all; voiceprint is a separate service with no link to Scene |
| *seen and now gone* | departures table exists | **no contract reads it** |
| *visit history* ("常去") | — | absent; nothing records where the robot has been sent |
| standing queries | — | absent; every contract is one-shot |

The three that matter most, in order:

1. **Horizontal relations.** `left_of` and friends are the single largest
   class of human referring language and Scene maintains none of them.
   Everything else on this list is a convenience next to that.
2. **Departures, exposed.** The table already distinguishes *ttl_pruned*,
   *operator_deleted*, *derived_cleared*, *epoch_flush* and
   *merged_duplicate*, with forwarding addresses. It answers GPSR category
   3 — "there is no milk" versus "I have not looked" — and nothing can read
   it.
3. **`count` and containment as contracts**, not as something each caller
   re-derives from `list_objects`.

---

## 6. What this says about ordering

Before more Scene features:

- **T1 belongs to somebody.** A robot that cannot answer "what can you do"
  fails the first sentence of every real interaction. Atlas knows every
  registered capability; Pilot has the list in its prompt. The gap is that
  no one decided whose job it is to say so out loud.
- **The wake phrase must stop becoming the task.** One line in Liaison's
  hands-free path; it makes the difference between 14 sessions of "你好"
  and 14 real requests.

Then, in Scene, in this order: horizontal relations; a contract over
departures; `count` and region containment; visit history for T6.

`find` and `go_to` exist and are now declared, but until horizontal
relations exist they can only disambiguate on distance and class — which is
the easy 20% of how people point at things.

---

## Sources

- Walker, Peng, Cakmak. *Neural Semantic Parsing with Anonymization for
  Command Understanding in General-Purpose Service Robots.* arXiv:1907.01115.
  Tables 1–2.
- RoboCup@Home `gpsr_command_generator`, category definitions.
- Li et al. *BEHAVIOR-1K.* arXiv:2403.09227. Survey of 1,461 respondents.
- Achlioptas et al. *ReferIt3D.* ECCV 2020. Nr3D/Sr3D sizes, human accuracy,
  Sr3D relation-type shares.
- Deployment logs, robot session 2026-09-20 17:32–17:46.

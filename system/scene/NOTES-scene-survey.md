# What published scene-graph systems store, and how they are queried

2026-09-20. A literature survey, written before adding anything else to
Scene. The question is not "what else could Scene hold" but "what have
systems that were actually deployed found it necessary to hold, and what
interface did they hand to whatever consumes it".

Every number below is from the paper named beside it. Retrieved via arXiv
(`export.arxiv.org/api/query`, `id_list`) on 2026-09-20; full texts read
from `arxiv.org/html` or the PDF where the HTML build was missing.

---

## 1. What they store

### The layer taxonomies agree more than they differ

| System | Layers, bottom to top |
|---|---|
| 3D Scene Graph (Armeni, ICCV'19) | camera → object → room → building |
| Hydra (RSS'22) | mesh → objects (+ agent poses) → places → rooms → building |
| SayPlan (CoRL'23) | object (movable) → asset (immovable) → room → floor, plus **pose** nodes and a **dynamic agent** node |
| HOV-SG (RSS'24) | object → room → floor |
| Clio (RA-L'24) | object → place → region |

Four recurring ideas, none of which Scene has all of:

- **A traversability layer distinct from objects.** Hydra's *places* are
  obstacle-free locations, "an edge between places denotes straight-line
  traversability". SayPlan's *pose* nodes do the same job and let it hand
  path planning to Dijkstra instead of the LLM. Scene has `reachable_by`
  as an object attribute, not a layer that can be planned over.
- **Immovable vs movable as a type distinction, not an attribute.**
  SayPlan splits *asset* from *object* precisely because you navigate to
  the first and manipulate the second.
- **An agent node.** Hydra tracks agent poses; SayPlan carries a dynamic
  agent node; OVSG makes *Agent* one of its three entity types outright.
  Scene has the robot as an object with `is_robot`, and no notion of a
  person at all.
- **Rooms/regions as first-class nodes with their own edges.** Scene has
  regions but they are operator-drawn annotations, not graph nodes other
  things attach to.

### What a node carries

SayPlan's node format, verbatim:

```
{name: coffee_machine, type: asset, location: kitchen,
 affordances: [turn_on, turn_off, release], state: off,
 attributes: [red, automatic], position: [2.34, 0.45, 2.23]}
```

ConceptGraphs — which is what Scene runs — stores per node: "object id,
bounding box extents, bounding box center, object tag, and caption", plus
the point cloud and a semantic feature vector.

The gap is `affordances` and `state`. SayPlan's whole verification step
(`verify_plan`) works by checking a plan against exactly those two fields:
it returns "cannot pick banana" when the fridge containing it is closed.
Without them there is nothing to verify a plan against.

### Edges

ConceptGraphs builds edges by 3D bounding-box IoU, prunes with a minimum
spanning tree, and then asks an LLM per surviving edge to "describe the
likely spatial relationship between the objects, such as 'a on b' or 'b in
a'". **No fixed vocabulary** — the paper explicitly notes the LLM may
"extend the nominal edge type" to things like "a backpack *may be stored
in* a closet".

That is the origin of Scene's relation set, and also of its limits: the
edges that exist are the ones an LLM volunteered for object pairs that
already overlapped in 3D. Relations between objects that do *not* overlap
— which is what "left of" and "across from" are — never get proposed.

Reported edge precision on Replica: 0.88–0.91, with node precision 0.71
(CG) / 0.61 (CG-D), over 23–60 valid objects per scene.

---

## 2. How they are queried

### Nobody passes the whole graph

This is the strongest single finding, and it is the opposite of what Scene
currently offers.

**SayPlan** gives the LLM a four-function API over the graph and nothing
else:

```
collapse()            → graph with only the top level exposed
expand(node_name)     → reveal the nodes one level below node_name
contract(node_name)   → hide them again
verify_plan(plan)     → forward-simulate against predicates/states/affordances
```

The collapse alone reduces the token representation "by ≈ 80%", and up to
**82.1%** for the largest scene. Scale tested: an office of **37 rooms and
150 assets and objects**, and a three-storey house with **28 rooms and 112
objects**; 90 tasks across four difficulty levels.

Semantic search success — finding the right subgraph at all:

| | Office simple | Office complex | Home simple | Home complex |
|---|---|---|---|---|
| Human | 100% | 100% | 100% | 100% |
| SayPlan (GPT-4) | 86.7% | 73.3% | 86.7% | 73.3% |
| SayPlan (GPT-3.5) | 6.6% | 0.0% | 0.0% | 0.0% |

**EmbodiedRAG** reaches the same conclusion from the retrieval side:
passing the 3DSG as-is "quickly becomes infeasible due to input token count
limits and attentional biases", and subgraph retrieval cuts input tokens
"by an order of magnitude" and planning time "up to 70%".

Scene's `list_objects` returns every object with no filter, no scoping, no
level. Its own contract file says "No filters, no scoping; the LLM filters
client-side." That is the design both of these papers report as the thing
that stops working.

### The query is a graph, not a string

**OVSG** (CoRL'23) is the closest published thing to what `find` is trying
to be. Three entity types — **Object, Agent, Region** — and two relation
families, spatial and abstract. A free-form query is parsed by an LLM into
a small query *graph* of entity nodes and relation edges, then matched
against the scene graph.

Their evaluation splits queries into two kinds, and the split matters:

- **object-only** — no agent or region constraint
- **whole** — "inherently contain a mix of agent, region, and object
  preferences", e.g. *"I want to find Tom's bottle in the laboratory"*

Top-1 grounding success on whole queries:

| Dataset | scenes / queries | OVSG-L | best baseline |
|---|---|---|---|
| ScanNet | 312 / ~62,000 | **58.85** | OVIR-3D 38.56 |
| DOVE-G | 8 / 4,000 | **54.25** | OVIR-3D 35.5 |
| ICL-NUIM | — / 359 whole, 190 object-only | **74.09** | ConceptFusion 39.28 |

Two readings for us. First, context-aware matching beats
semantic-similarity retrieval by ~20 points — so the graph is worth
having. Second, the best number on realistic queries is **59%**, which
means a clarification turn is part of the normal path, not an error path.

**Transcrib3D** makes the opposite architectural bet and it is worth
noting: it serialises the 3D scene to *text* and lets the LLM reason over
it, "sidestep[ping] the need to learn shared representations". That is
roughly what Scene's `get_object_context` does today, and it does reach
SOTA on reference resolution — so the text-serialisation route is not
obviously wrong, it just doesn't scale past the token limit that SayPlan
and EmbodiedRAG are both working around.

---

## 3. What the tasks actually look like

**SG3D** (2024) is the benchmark closest to "what does a user actually ask
over a session": **22,346 tasks, 112,236 steps, 4,895 real scenes**, built
from ScanNet (693 scenes / 3,174 tasks), 3RScan (472/2,194), ARKitScenes
(1,575/7,395), HM3D (2,038/9,036), MultiScan (117/547).

- average **5.03 steps per task**, 70.5 words
- most common task verbs: *prepare*, *organize*; most common actions:
  *walk*, *place*
- most common targets: cabinets, tables, chairs, sinks, beds, shelves

Results:

| Method | step acc. | task acc. |
|---|---|---|
| PQ3D | 57.3% | 26.8% |
| 3D-VisTA | 60.9% | 30.6% |
| LEO (3D LLM) | 62.8% | 34.1% |
| GPT-4 **with ground-truth labels** | 73.4% | **46.6%** |

The last row is the one that matters. Given perfect perception, a
state-of-the-art LLM completes fewer than half of these tasks, because a
task is five steps and each step re-grounds against a scene that the
previous step changed. Per-step accuracy is not the metric; carrying
grounding across steps is.

---

## 4. Task-driven granularity — the direct answer to "design Scene from the application"

**Clio** (RA-L'24) poses exactly the question: *"what is the right
granularity for the objects the robot has to include in its map?"* and
answers that "such a choice is intrinsically task-dependent". It takes a
list of natural-language tasks **at the start of operation** and uses an
Information-Bottleneck formulation to cluster 3D primitives into only the
objects and regions those tasks need.

The effect is large:

- Cubicle scene: **1,880 primitives → 84 objects**
- F1 on task-relevant object recovery: **Clio 0.80**, Khronos 0.42,
  **ConceptGraphs 0.25**
- ~0.30 s per frame, onboard
- Demonstrated granularity switching: "get all condiment packets" clusters
  the sauces into one object; "get specific sauce types" splits them

ConceptGraphs at 0.25 against Clio at 0.80 on *task-relevant* objects is
the number that should decide our next move. Scene currently runs
ConceptGraphs with a fixed threshold, so its granularity is whatever the
detector happened to produce — which is why the object list fills with
`picture_frame ×6` nobody will ever refer to, while the one chair somebody
does refer to is split across two records.

**HOV-SG** reports the compression side of the same idea: a floor/room/
object hierarchy gives a "75% reduction in representation size compared to
dense open-vocabulary maps" while beating baselines at object, room and
floor level.

---

## 5. Where Scene stands against this

| What the literature converged on | Scene today |
|---|---|
| Layered graph: object / place / region / floor | objects + operator-drawn regions; no place layer |
| Traversability as a planned-over layer (Hydra, SayPlan) | `reachable_by` edge on objects |
| Agent/person as an entity type (OVSG, Hydra, SayPlan) | robot is an object flag; no person |
| `affordances` + `state` per node (SayPlan) | neither; nothing to verify a plan against |
| Query API: collapse / expand / contract / verify (SayPlan) | `list_objects` returns everything, unscoped |
| Subgraph retrieval before the LLM (EmbodiedRAG) | none; the whole snapshot goes to Pilot |
| Query parsed into an entity+relation graph (OVSG) | `find` parses a flat query; relations are optional filters |
| Task-driven granularity (Clio) | fixed detector thresholds |
| Relations beyond containment/support | `near / on_top_of / under / inside / contains` — no horizontal relations |

## 6. What the survey implies, in order

1. **A query API instead of a dump.** SayPlan's four operations are small
   and the reduction they buy is measured: ~80% of tokens, and GPT-3.5 goes
   from 0% to unusable without them. `list_objects` with "the LLM filters
   client-side" written into its contract is the documented failure mode.
2. **A place layer.** It is what lets SayPlan delegate path planning and
   what gives Hydra rooms for free. We have the occupancy grid already; we
   throw the topology away.
3. **Horizontal relations.** ConceptGraphs only proposes edges for pairs
   whose boxes overlap, so we inherited containment and support and nothing
   else. Nothing about "left of" requires an LLM — it is a projection into
   the viewer's frame, and it is the majority of how people point.
4. **Affordance and state fields**, even if only ever written by hand at
   first. Without them `verify_plan` has nothing to check and every plan
   failure is discovered by driving into it.
5. **Task-driven granularity is a later move, but it is the one with the
   biggest measured gap** (0.25 → 0.80). It also needs §"what is the
   application" answered first, because Clio's input *is* the task list.

---

## Provenance

Retrieved 2026-09-20 via `https://export.arxiv.org/api/query?id_list=...`
(one call, 11 ids), then full text per paper.

| Paper | arXiv | venue |
|---|---|---|
| 3D Scene Graph | 1910.02527 | ICCV 2019 |
| SceneGraphFusion | 2103.14898 | CVPR 2021 |
| Hydra | 2201.13360 | RSS 2022 |
| SayPlan | 2307.06135 | CoRL 2023 |
| OVSG | 2309.15940 | CoRL 2023 |
| ConceptGraphs | 2309.16650 | ICRA 2024 |
| HOV-SG | 2403.17846 | RSS 2024 |
| Clio | 2404.13696 | RA-L 2024 |
| Transcrib3D | 2404.19221 | CoRL-W 2023 |
| SG3D | 2408.04034 | 2024 |
| EmbodiedRAG | 2410.23968 | 2024 |

Warnings: `arxiv.org/html` returned 404 for the v2 builds of OVSG, SayPlan
and ConceptGraphs; OVSG and ConceptGraphs were read from v1 and SayPlan
from the PDF, so version-specific numbers may differ from the latest
revision. ReferIt3D's arXiv id was guessed wrong in the first batch
(1910.02527-era id collided with an unrelated proceedings entry) and its
figures here come from the earlier application survey rather than a fresh
retrieval.

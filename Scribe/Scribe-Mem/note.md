
TODO:

后续再考虑从日志中 get 到记忆的结构 - 自动化记忆的构建 - 目前先妥协

# 7.6

https://www.modelscope.cn/models/Qwen/Qwen3-Embedding-0.6B/summary - 替换成 0.6B的 - 暂时不用 Embeding 只用文本

考虑到Embeding 在少量文本中不太好用，搜索相关内容（将存储的知识图谱和问题）和用户问题 一同交给 LLM 让其返回相关问题的事件链（事件节点），即 hybrid_search 使用 LLM 去执行 search ；但保留 embeding 的接口，并保留相应的代码，默认不开启；将相关的修改 补充到 @Scribe/Scribe-Mem/step.md

添加 LLM 搜索记忆的实现，默认不用 Embeding；搜索相关内容（将存储的知识图谱和问题，不包括embeding的信息）和用户问题 一同交给 LLM 让其返回相关问题的事件链（事件节点），即 hybrid_search 使用 LLM 去执行 search ，在没有开启 Embeding 的情况下默认不使用 Embeding ;将修改补充的内容补充到 @Scribe/step.md 中

从日志中读取相关信息并建图-交由LLM执行
Demo3 去上一个位置 ... - 基础实现
录制 视频 Webots
场景信息 2 存储的记忆

韩博，基于刚刚的说法检索过程先用LLM做（先不用embeding），满足现在直接用文本的说法（当前少量记忆也不存在记忆爆炸的问题），然后后续的工作，包括记忆的提炼，记忆的遗忘等，先用 LLM 基于文本做一个基础版本的实现，以尽快实现一个场景的演示；场景的话跟张博商量了一下暂定错误场景（碰撞，抓取失败等）的记忆展示和地点记忆（小车移动到某个位置，不过这个可能跟语义地图有点冲突）；当前这个版本的话如果没啥问题我就先不写那个测试了（然后能合并的话就先合并上去），在下次基于文本的版本中我加上这部分测试

记忆这边其实现在就一个需求
就是让小车在四处转然后回来
嗯嗯
然后问小车问题，比如刚刚的窗户有没有关
问题是巡检完后现场问的
OK，就是刚刚巡检场景的对吧
这个其实和之前的区别就是，你要持续输入image
然后把“窗户有没有关”在memory查到相关image或记录，然后甚至可以直接返回image，pilot支持image message，然后让大模型看
大概就是这个流程，memory可以返回不同模态的数据，然后 pilot 这边主要通过识别 base64
自动把base64图片创建为一个 image message 给 VLM
这个你可以参考 camera primitive 相关的接口设计，那个就是直接返回带图片的结构
或者就是不改 pilot，你先内部内置一个 VLM，这样最快

OK我理解了，我这里记忆大概的流程是，检测到的物体会以文本的形式存储到记忆中，然后后续检索的时候问就返回，暂时还没有多模态这个存储，不过要是以图片的形式存在特定路径然后查询也可以做；不过如果能直接基于文本做这边可能更省事，唯一问题是文本可能没有描述那个物体要问的特征，所以还是内置VLM比较省事；那我把这个场景加上，尽快实现一下

Demo 错误场景展示 

## 下周

API  - 谁获取截图 - 从哪获取截图 - 如何触发获取截图的操作 - 根据物体解析来进行图片的保存
Struct LLM记忆化（不用Embeding）
Struct 记忆存储图片路径 - 参考 camera primitive 相关的接口设计，那个就是直接返回带图片的结构 OR 内置 VLM 对图片解析

Demo 巡检保存 图片 -> 基于图片问问题 - 基于Webots来做
Demo 错误记忆
Demo 地点记忆

1.图片存到 services/memory/data/images/{node_id}/ 中
2.观察并记忆在识别模块识别到物体后，即检测到物体后，触发记忆节点的保存和图像的保存
3.VLM直接用 pilot 的 vlm 配置

-------------------------------------

PTDL 规划获取，将规划转成节点进行存储

可以参考https://github.com/syswonder/robonix/blob/dev/testing/SCENARIO_SPEC.md，加一个 yaml 的 testcase 测试点，测试那几个接口 hybrid_search 等，相关修改和代码补充到 @Scribe/Scribe-Mem/step.md 中

-------------------------------------
  联调流程：完整巡检→记忆→VLM问答

  整体链路

  │    cap: tiago_camera.camera_snapshot → image_base64      │
  │    cap: scene.scene_list_objects     → objects[]         │
  │    cap: tiago_lidar.lidar_snapshot   → pointcloud        │

  ┌─ Step 4: VLM 看图问答 ──────────────────────────────────┐
  │  cap: memgraph.memgraph_hybrid_search                    │
  │  args:                                                   │
  │    data: {                                               │
  │      "query":"what was on the kitchen counter?",         │
  │      "tags":{"scene_type":"kitchen"},                    │
  │      "vlm_qa":true                                       │
  │    }                                                     │
  │  → 内部: 检索 MemoryNode → 加载图片 → VLM 看图 → 回答    │
  │  → 返回: {nodes:[...], vlm_answer:"There was a red cup"} │
  │  contract: robonix/service/memory/hybrid_search          │
  └──────────────────────────────────────────────────────────┘

  命令序列

  关键差异（memgraph vs memsearch 旧流程）

  ┌──────────┬─────────────────────────────────────┬──────────────────────────────────────────────┐
  │          │ memsearch（旧 patrol_observe.yaml） │                memgraph（新）                │
  ├──────────┼─────────────────────────────────────┼──────────────────────────────────────────────┤
  │ 写入     │ memory.memory_save(data="文本")     │ memgraph.memgraph_remember(data={JSON})      │
  ├──────────┼─────────────────────────────────────┼──────────────────────────────────────────────┤
  │ 图片     │ 不保存                              │ kv.image_base64 → 自动落盘                   │
  ├──────────┼─────────────────────────────────────┼──────────────────────────────────────────────┤
  │ 检索     │ memory.memory_search(data="关键词") │ memgraph.memgraph_hybrid_search(data={JSON}) │
  ├──────────┼─────────────────────────────────────┼──────────────────────────────────────────────┤
  │ VLM QA   │ 不支持                              │ vlm_qa: true → 内联 VLM 看图回答             │
  ├──────────┼─────────────────────────────────────┼──────────────────────────────────────────────┤
  │ 空间过滤 │ 不支持                              │ tags:{scene_type:"kitchen"}                  │
  └──────────┴─────────────────────────────────────┴──────────────────────────────────────────────┘


（后续再考虑从日志中 get 到记忆的结构）

观测当前场景，查看屋中有什么物体

查看当前记忆节点，看有什么记忆

转身



cap=tiago_camera.camera_snapshot args={}
cap=scene.scene_list_objects args={}
说明scene.scene_list_objects如何根据图像信息识别物体的，识别后的物体列表是什么

list_objects 读取已缓存的结果时，调用 memgraph_remember ，并通过参数激活，使得 memgraph_remember 调用  camera_snapshot 图片保存，别的情况不触发图片的保存


在hybrid_search 时，如果有图片就将用户问题和图片和相关的记忆一起发送给VLM做识别（先调用llm根据记忆图谱+用户问题找到相关的记忆节点，然后调用vlm根据记忆节点+用户问题去做回答）


mcp_tools.list_objects() 当 mcp_tools.list_objects() 时，触发一个记忆的保存，依照 perception_concept_graphs.py 中获取图片的途径，将图片进行保存，不在使用 scene 容器内的一个hook，这样是不是就不用实现容器内到容器外的调用了？？？？

也订阅一个


  关键代码位置

  ┌──────────────────────────┬─────────────────────────────────────────┬────────────────┐
  │           步骤           │                  文件                   │       行       │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ ROS 订阅 RGB/Depth/Lidar │ ingest/ros_subscribers.py:211           │ _subscribe()   │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ YOLO-World 开集检测      │ ingest/perception_concept_graphs.py:259 │ _load_models() │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ MobileSAM bbox 分割      │ 同上                                    │ 同上           │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ 3D 点云从 depth + 内参   │ 同上 detections_to_obj_pcd_and_bbox     │ -              │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ 跨帧关联 + 去重          │ 同上 merge_detections_to_objects        │ -              │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ 更新 ObjectRegistry      │ state/object_registry.py                │ -              │
  ├──────────────────────────┼─────────────────────────────────────────┼────────────────┤
  │ MCP 返回物体列表         │ mcp_tools.py:84                         │ list_objects() │
  └──────────────────────────┴─────────────────────────────────────────┴────────────────┘

  ROS 总线 还有这个 camera_snapshot

-----------

LLM 查询 记忆节点， piolt 调用VLM查询物体其他信息(或者是由LLM判定是否需要调用 VLM 去查询)

根据日志 @/home/hyl/robonix/examples/webots/rbnx-boot/logs/memgraph.log 和 @/home/hyl/robonix/examples/webots/rbnx-boot/logs/pilot.log 说明vlm调用是否成功调用，并说明为什么vlm没有给出可能的植物的种类

没有显式开启embeding的情况下，取消 embeding ； 

rbnx 启动后删除 data/images 下的节点和图片 和 memory/graph_store.json 


不该变当前的 scene list 的 场景存储；新增 物体存储 目标是 小车 巡查过程中，有新物体出现就保存图片，实现方法是 后台有个进程不断查看 scene_list 中有什么物体;设计为一个物体存储成一个图片

根据日志 @/home/hyl/robonix/examples/webots/rbnx-boot/logs/memgraph.log 和 @/home/hyl/robonix/examples/webots/rbnx-boot/logs/pilot.log 说明为什么转身后没有保存相应的记忆，当前 scene.scene_list_objects({}) 可以被正常调用，可以成功保存记忆，说明以上情况的原因

注释掉相应代码，  scene.scene_list_objects({}) 成功实现保存，将此次新增功能补充到 @Scribe/Scribe-Mem/step.md 中，并注明错误原因，然后修复

rbnx build -p ../../system/scene

根据日志 @/home/hyl/robonix/examples/webots/rbnx-boot/logs/memgraph.log 和
  @/home/hyl/robonix/examples/webots/rbnx-boot/logs/pilot.log 说明为什么转身后没有保存相应的记忆，当前
  scene.scene_list_objects({}) 不能被正常调用，同时成功保存记忆，说明以上情况的原因；为什么改了还是错误的？？？已经
  rbnx build scene 了，说明原因

----

删除重复的物体， 重复物体不应该保存

LLM 检索，在合适的时候调用 VLM

------

查看当前场景中物体

巡查场景

根据记忆查看盆栽植物颜色

根据记忆查看盆栽植物有几片叶子

根据记忆猜测盆栽植物类型

根据当前记忆中保存的图片查看盆栽是什么种类的植物


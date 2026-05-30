# Soma — body state, primitive abstraction

One of the 12 Robonix system components. The runtime's model of "what
hardware this body has, where each device sits, and what each device
can do." Owns the URDF, the joint / link / sensor tree, and the
mapping from physical devices to capability instances.

**Status — v0.1 stub.** Not yet implemented as a service.

In v0.1 the body model lives in two places:

- per-deploy URDF + a side-launch of `static_transform_publisher`
  (see for example `ranger_mini_deploy/side_launch/static_tf.launch.xml`),
- primitive packages that register themselves directly with atlas
  (`rbnx/primitive/chassis/driver`, `…/camera/rgb`, etc.).

When Soma lands it will:

- enumerate hardware (cameras, lidars, arms, chassis, end-effectors)
  at boot,
- publish a single URDF + TF tree that downstream services
  ([scene](../scene/), navigation, manipulation) consume,
- broker the "device id → capability instance(s)" mapping so a single
  physical camera shows up as one [scene](../scene/)-visible device
  rather than three independent rgb / depth / extrinsics primitives.

See the whitepaper §3 *L1 具身场景层* and §4 *本体抽象 (Soma)*.

# Third-party assets: VRX WAM-V

Upstream project: `osrf/vrx`

- Repository: https://github.com/osrf/vrx
- Branch audited: `jazzy`
- Exact commit: `7609d1bd90ce7edb29d040a082f949e8b089c864`
- License: Apache License 2.0
- License copy: `LICENSE-VRX.txt`

## Imported files

The build imports these immutable upstream files from the exact commit above and installs them locally under this model's `meshes/` directory:

```text
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V-Base.dae
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Albedo.png
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Normal.png
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Roughness.png
vrx_urdf/wamv_description/models/WAM-V-Base/mesh/WAM-V_Metalness.png
```

The numerical WAM-V collision geometry in `model.sdf` is derived from:

```text
vrx_urdf/wamv_description/urdf/wamv_base.urdf.xacro
```

## Project modifications

The upstream WAM-V visual mesh and PBR maps are not modified. This project supplies its own SDF wrapper and intentionally omits VRX propulsion, thruster, buoyancy, hydrodynamics, wind, sensors and competition plugins. A fixed 2.4 m × 2.4 m UAV landing platform, supports, ArUco visuals and Ground Truth odometry plugin are project additions.

The official `wamv/base_link` geometry is mapped to this model's `vessel_body` canonical frame. The added `landing_deck` frame is fixed at `[0, 0, 1.80] m` with identity rotation.

## Runtime dependency policy

No Gazebo Fuel URI or user cache is used. CMake performs a build-tree-only Git partial clone of the upstream repository, verifies the exact commit, exports only the listed blobs, validates their expected byte sizes, and installs them into the local `moving_deck_sim` share. Running the built marine environment is offline with respect to VRX assets.

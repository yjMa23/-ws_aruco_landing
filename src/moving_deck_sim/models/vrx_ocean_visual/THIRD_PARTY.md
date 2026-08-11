# Third-party components: VRX dynamic wave visual

- Repository: https://github.com/osrf/vrx
- Branch audited: `jazzy`
- Exact commit: `7609d1bd90ce7edb29d040a082f949e8b089c864`
- License: Apache License 2.0
- License copy: `../vrx_wamv_landing/LICENSE-VRX.txt`

Imported from the pinned commit at build time:

```text
vrx_gz/models/coast_waves/meshes/waterlow.dae
vrx_gz/models/coast_waves/materials/programs/GerstnerWaves_vs_330.glsl
vrx_gz/models/coast_waves/materials/programs/GerstnerWaves_fs_330.glsl
vrx_gz/models/coast_waves/materials/textures/wave_normals.dds
vrx_gz/models/coast_waves/materials/textures/skybox_lowres.dds
vrx_gz/src/WaveVisual.cc
vrx_gz/src/WaveVisual.hh
vrx_gz/src/Wavefield.cc
vrx_gz/src/Wavefield.hh
```

The imported files are not modified. `WaveVisual.cc` and `Wavefield.cc` are compiled together into the local `libWaveVisual.so`; the SDF only supplies project-selected visual CWR parameters. No VRX buoyancy, hydrodynamics, wind, propulsion, competition or scoring plugin is copied or enabled.

The built runtime resolves the mesh, shaders and textures from the local `moving_deck_sim` install share and the plugin from the local package `lib/` directory. It has no Gazebo Fuel or user-cache dependency.

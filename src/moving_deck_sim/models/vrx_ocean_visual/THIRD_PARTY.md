# Third-party asset: VRX wave normal texture

- Repository: https://github.com/osrf/vrx
- Branch audited: `jazzy`
- Exact commit: `7609d1bd90ce7edb29d040a082f949e8b089c864`
- License: Apache License 2.0
- License copy: `../vrx_wamv_landing/LICENSE-VRX.txt`

Imported at build time:

```text
vrx_gz/models/coast_waves/materials/textures/wave_normals.dds
```

The texture is not modified. This project supplies its own visual-only plane and PBR material. It intentionally does not copy or enable VRX `WaveVisual`, `Wavefield`, coast water mesh, buoyancy, hydrodynamics or wind plugins in Marine M2.

The built runtime model uses only the local installed copy under `materials/textures/` and has no Fuel or user-cache dependency.

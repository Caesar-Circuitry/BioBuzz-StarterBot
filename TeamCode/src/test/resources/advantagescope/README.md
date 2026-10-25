# BioBuzz sim AdvantageScope assets

Installed automatically into AdvantageScope's user-assets folder by
`BioBuzzAssets.install()` every time `SimulateMecanumBase` starts (only changed
files are copied). Restart AdvantageScope once after the first install.

| Folder | What it is |
|---|---|
| `Field3d_BioBuzzSim` | AdvantageScope's official 2026-2027 field with both hives removed. Shows up as "2026-2027 Field (BioBuzz Sim)". |
| `Robot_BioBuzzRedHive` | The red hive cut out of the same model. Origin on its pivot axis, drawn level. |
| `Robot_BioBuzzBlueHive` | The blue hive, same. |

The sim logs `BioBuzz/Hive/RED/pose` and `BioBuzz/Hive/BLUE/pose` (Pose3d: pivot
position + pitch = hive angle) and sets up a "3D BioBuzz" tab that shows them on
the hive-less field.

## Regenerating

If AdvantageScope updates its 2026-2027 field, copy the new `config.json`,
`model.glb` and `model_0..2.glb` from its `autoAssets` folder somewhere, then:

    pip install pygltflib numpy
    npm install -g @gltf-transform/cli
    python3 build_assets.py path/to/model.glb path/to/folder-with-config-and-model_N
    # then for each assets/*/raw.glb:  gltf-transform prune raw.glb model.glb

and replace these folders with the ones in `assets/`.

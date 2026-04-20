# NS3 Debug Record

## Date

2026-04-20

## Goal

Check whether previous NS3 changes in `/home/sakakibara/Workspace/carla-ns3-co-simulation` were safely stored inside the repository, because this project installs an external `ns-3-dev` tree and direct edits there may not be committed to git. If any NS3 core files had been modified outside the repository mirror, move them under the repository so future installs do not lose them.

## References Used

- `C:/Workspace/OpenCDA/NS3_bug.md`
- Git history of `carla-ns3-co-simulation`
- Current repository tree under `ns3/src`, `ns3/vanet`, and external `ns-3-dev`

## Inspection Process

1. Read `C:/Workspace/OpenCDA/NS3_bug.md` and extracted the previously documented fix points.
2. Checked git history in `/home/sakakibara/Workspace/carla-ns3-co-simulation`.
3. Confirmed that the documented key fixes were already committed inside the repository mirror:
   - `ns3/src/nr-spectrum-phy.cc`
   - `ns3/src/nr-sl-ue-mac-scheduler-manual.cc`
   - `ns3/src/nr-sl-ue-mac-scheduler-fixed-mcs.cc`
   - `ns3/src/nr-sl-ue-mac.h`
   - `ns3/vanet/main.cc`
   - `ns3/vanet/cam-application.cc`
4. Checked `installns3.sh` and verified the intended structure:
   - `ns3/vanet/*` is linked into `ns-3-dev/scratch/vanet`
   - selected files under `ns3/src/*` are linked into `ns-3-dev/contrib/nr/model`
5. Verified that the currently important NR files are already symlinked back to the repository mirror, so edits to them are not at risk.
6. Checked the external `ns-3-dev` git status and found 5 modified tracked files under `src/lte/model/` that were not mirrored into the repository.

## Risk Found

The following files in external `ns-3-dev/src/lte/model/` had local modifications and were not protected by the repository mirror:

- `epc-ue-nas.cc`
- `lte-net-device.cc`
- `lte-pdcp.cc`
- `lte-rlc-um.cc`
- `lte-ue-rrc.cc`

This means those changes could be lost if `ns-3-dev` were re-cloned or rebuilt from scratch.

## Contents of the External LTE Changes

The 5 external changes were mostly debug instrumentation, plus one functional buffer-size change:

- `epc-ue-nas.cc`: added `[NAS_RECV_DATA]` logging
- `lte-net-device.cc`: added `[LTE_NETDEVICE_RECV]` logging
- `lte-pdcp.cc`: added `iostream` include and `[NRSL_RLC_TO_PDCP]` logging
- `lte-rlc-um.cc`: added `iostream`, raised `m_maxTxBufferSize` from `10 * 1024` to `99 * 1024 * 1024`, and added `[NRSL_RLC_RX]` / `[NRSL_RLC_DROP]` logging
- `lte-ue-rrc.cc`: added `[NRSL_PDCP_TO_RRC]` logging

## Actions Taken

### 1. Preserved external LTE changes inside the repository

Created a new mirror directory:

- `ns3/src/lte-model/`

Copied the 5 modified LTE files into that directory:

- `ns3/src/lte-model/epc-ue-nas.cc`
- `ns3/src/lte-model/lte-net-device.cc`
- `ns3/src/lte-model/lte-pdcp.cc`
- `ns3/src/lte-model/lte-rlc-um.cc`
- `ns3/src/lte-model/lte-ue-rrc.cc`

### 2. Replaced risky external files with symlinks

Re-pointed the following external files so they now reference the repository mirror instead of staying as standalone local edits:

- `ns-3-dev/src/lte/model/epc-ue-nas.cc`
- `ns-3-dev/src/lte/model/lte-net-device.cc`
- `ns-3-dev/src/lte/model/lte-pdcp.cc`
- `ns-3-dev/src/lte/model/lte-rlc-um.cc`
- `ns-3-dev/src/lte/model/lte-ue-rrc.cc`

### 3. Updated installation logic

Updated `installns3.sh` so future installs will recreate the mirror correctly.

Old behavior:

- linked `ns3/vanet/` to `ns-3-dev/scratch/`
- linked `ns3/src/*` to `ns-3-dev/contrib/nr/model/`
- linked `ns3/cmake/*` to `ns-3-dev/contrib/nr/`

New behavior:

- links `ns3/vanet/` to `ns-3-dev/scratch/`
- links `ns3/src/*.cc` and `ns3/src/*.h` to `ns-3-dev/contrib/nr/model/`
- links `ns3/src/lte-model/*` to `ns-3-dev/src/lte/model/`
- links `ns3/cmake/*` to `ns-3-dev/contrib/nr/`

## Result

After the migration:

- the previously documented NR/vanet fixes are still inside the repository mirror
- the 5 risky LTE core modifications are no longer only stored in external `ns-3-dev`
- future `installns3.sh` runs will recreate both the NR mirror and the LTE mirror
- if NS3 core code must be modified going forward, it should be modified under the repository mirror, not directly in `ns-3-dev`

## Rule Going Forward

If NS3 core files need to be changed, the repository copy must be edited.

For the current repository, that means:

- NR-related mirrored files: edit under `ns3/src/`
- LTE core files preserved in this migration: edit under `ns3/src/lte-model/`
- VANET bridge code: edit under `ns3/vanet/`

Do not directly edit standalone files inside `ns-3-dev` unless they are first mirrored into the repository and wired through `installns3.sh`.

## Verification Performed

Verified that:

- `ns-3-dev/scratch/vanet` points to `ns3/vanet`
- key NR files under `ns-3-dev/contrib/nr/model/` point to `ns3/src/`
- the 5 LTE files under `ns-3-dev/src/lte/model/` now point to `ns3/src/lte-model/`
- `installns3.sh` contains the new LTE mirror symlink rule

## Notes

This step did not rebuild NS3 or rerun the co-simulation. The work here focused on preserving previous source modifications and preventing future loss.

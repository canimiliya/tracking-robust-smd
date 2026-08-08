# S0 official SMD bootstrap reproduction log

## Identity

- TASK_ID: S0-R0-OFFICIAL-SMD-BOOTSTRAP-R1
- DATE: 2026-08-08 19:09:19 +08:00
- LOCAL_ROOT: D:/Desktop/my_project/TR-SMD
- OLD_REPO: https://github.com/canimiliya/TR-SMD
- NEW_REPO: https://github.com/canimiliya/tracking-robust-smd.git
- REPOSITORY_RENAME: PASS; owner canimiliya; visibility PUBLIC
- BRANCH: repro/smd-official
- UPSTREAM: https://github.com/RAISELab-atUVA/Diffusion-MRMP.git
- UPSTREAM_BRANCH: main
- UPSTREAM_HEAD: c87fc76044b350a37fcea7afc468c13c8371a237
- BASELINE_TAG: smd-official-import -> c87fc76044b350a37fcea7afc468c13c8371a237
- START_HEAD: c87fc76044b350a37fcea7afc468c13c8371a237

## Official source audit

- Required paths README.md, LICENSE, requirements.txt, setup.py, deps/, scripts/, scripts/inference/, smd/, and is_collision.py: present.
- OFFICIAL_INFERENCE_ENTRYPOINT: scripts/inference/launch_smd_composite_experiment.py
- OFFICIAL_COLLISION_ENTRYPOINT: is_collision.py
- README environment: Rocky Linux 8.10 tested, Python 3.8.20, CUDA 12.1.
- README checkpoint source: Google Drive file ID 1M0hfM5TlY45mzMxoyaeslvohSC2e74Yk, data_checkpoints.tar.gz.

## Commands run

- gh auth status
- gh repo rename --help
- gh repo rename -R canimiliya/TR-SMD tracking-robust-smd --yes
- gh repo view canimiliya/tracking-robust-smd --json name,owner,visibility,url,defaultBranchRef
- git clone https://github.com/RAISELab-atUVA/Diffusion-MRMP.git .
- git status, git branch --show-current, git rev-parse HEAD, git log -5 --oneline
- git remote rename origin upstream
- git remote add origin https://github.com/canimiliya/tracking-robust-smd.git
- git fetch upstream
- git fetch origin
- git switch -c repro/smd-official
- git tag smd-official-import c87fc76044b350a37fcea7afc468c13c8371a237
- conda create -n smd python=3.8.20 -y
- conda install -n smd patchelf -y (failed: no win-64 package)
- conda run -n smd python -m pip install setuptools==70.2.0
- conda run -n smd python -m pip install torch==2.1.2 torchvision==0.16.2 torchaudio==2.1.2 --index-url https://download.pytorch.org/whl/cu121
- conda run -n smd python -m pip install -r requirements.txt (failed at triton==2.1.0)
- conda install -n smd -c conda-forge ipopt -y
- Official fixed dependencies excluding only blocked Hydra==2.5 and triton==2.1.0: installed.
- pip install -e deps/torch_robotics --no-deps
- pip install -e deps/experiment_launcher --no-deps
- pip install -e deps/motion_planning_baselines --no-deps
- pip install -e . --no-deps
- conda run -n smd python -m gdown --fuzzy ... -O data_checkpoints.tar.gz
- tar -xzf data_checkpoints.tar.gz
- conda run -n smd python launch_smd_composite_experiment.py --start_index 0 --end_index 1
- conda run -n smd python is_collision.py
- conda run -n smd python -m pip freeze
- conda list -n smd

## Results

- INSTALL_STATUS: BLOCKED_NATIVE_WINDOWS_DEPENDENCY
- CHECKPOINT_STATUS: DOWNLOADED_AND_EXTRACTED
- CHECKPOINT_FILE_SIZE_BYTES: 609486811
- CHECKPOINT_SHA256: 5FB165686FA55E8955D842FA167B52ACD93A03AA513CD60787FEDF877B51689B
- IMPORT_STATUS: PASS_WITH_UNMET_REQUIREMENTS; torch, torchvision, torchaudio, smd, torch_robotics, experiment_launcher, mp_baselines, cholespy, scipy, cv2, pyomo, projection, and inference module imported.
- SMOKE_STATUS: FAIL; official inference stopped at line 105 with NameError: name map_name is not defined, before checkpoint loading/instance processing.
- COLLISION_SMOKE_STATUS: FAIL; is_collision.py stopped because scripts/inference/results_test does not exist.
- PIP_CHECK: No broken requirements found for installed distributions.

## Blockers and interpretation

1. Full requirements.txt is not installable on native Windows: triton==2.1.0 has no matching Windows distribution; Hydra==2.5 requires Unix header unistd.h during build.
2. patchelf is unavailable from the tested win-64/defaults channel.
3. NVIDIA driver exposes CUDA 13.0 and CUDA is enumerated, but the fixed PyTorch 2.1.2+cu121 build warns that RTX 5060 Ti sm_120 is unsupported. This is not evidence of usable GPU inference.
4. Official inference script has an unmodified source bug: map_name is referenced but not defined for the default three-agent path.
5. No algorithm source files were changed. No TR-SMD innovation, tuning, refactor, benchmark, or paper experiment was performed.

## Scope decision

This bootstrap is BLOCKED, not ready. The official Git history, remotes, branch, baseline tag, source provenance, partial environment, and checkpoint/data provenance are preserved. Do not start S1 or compatibility/source repair without higher-level authorization.

## S0-R1 native Blackwell remediation

- TASK_ID: S0-R1-NATIVE-BLACKWELL-COMPATIBILITY-R1
- BASE_BRANCH: repro/smd-official
- START_HEAD: 7b9e780547846d6455d00f84b912748bb2fc148d
- REMEDIATION_BRANCH: fix/s0-native-blackwell-compat
- OFFICIAL_BASELINE_TAG: smd-official-import -> c87fc76044b350a37fcea7afc468c13c8371a237
- DATE: 2026-08-08

### Dependency and GPU gates

- A separate `smd-blackwell` Conda environment was created with Python 3.9.25. The existing `smd` Python 3.8.20 environment was preserved unchanged as the official-version attempt.
- Official fixed stack: Python 3.8.20, torch 2.1.2+cu121. Compatibility stack: Python 3.9.25, torch 2.7.1+cu128, torchvision 0.22.1+cu128, torchaudio 2.7.1+cu128 from the official PyTorch cu128 index.
- GPU gate: PASS. `CUDA_AVAILABLE=True`, device `NVIDIA GeForce RTX 5060 Ti`, capability `(12, 0)`, CUDA tensor allocation PASS, 1024x1024 CUDA matmul PASS, finite result True. No `sm_120` unsupported warning was emitted.
- IPOPT 3.14.19 from conda-forge was available. Core imports passed for torch, torchvision, torchaudio, smd, torch_robotics, experiment_launcher, mp_baselines, cholespy, numpy, scipy, pyomo, safetensors, transformers, cv2, and the inference module when launched from `scripts/inference`.
- `pip check` is not clean by design and is recorded as a failure boundary: SMD metadata declares Hydra==2.5 and triton==2.1.0, both intentionally skipped after runtime audit, and both SMD/urdfpy metadata declare networkx==2.2 while the compatibility environment uses networkx==2.8.8.

### Runtime usage audit

- `HYDRA_RUNTIME_USAGE=DECLARED_BUT_NO_DIRECT_RUNTIME_USAGE`: no import or call found in `smd/`, `deps/`, or `scripts/`; only `requirements.txt` and package metadata declare it. The native Windows build failed on `unistd.h`.
- `TRITON_RUNTIME_USAGE=DECLARED_BUT_NO_DIRECT_RUNTIME_USAGE`: no import or call found in `smd/`, `deps/`, or `scripts/`; the pinned 2.1.0 package has no native Windows distribution.
- `PATCHELF_RUNTIME_USAGE=DECLARED_BUT_NO_DIRECT_RUNTIME_USAGE`: only the README install declaration at line 53; no runtime call. The tested win-64/defaults channel has no package.
- `networkx` is a direct runtime dependency through `deps/torch_robotics/torch_robotics/torch_kinematics_tree/geometrics/skeleton.py` and `.../models/utils.py`. The official 2.2 package raised `ImportError: cannot import name 'gcd' from 'fractions'` under Python 3.9. The minimal dependency-only override to 2.8.8 allowed the official path to continue; no source code was changed for this issue.

### Official bug reproduction and authorized patch

- Before source modification, the new environment reproduced the upstream failure exactly: `launch_smd_composite_experiment.py:105`, `NameError: name 'map_name' is not defined`.
- The only source patch changes the three, six, and nine-agent init trajectory filename references from `map_name` to the already parsed `args.map_name`. This is an entrypoint variable binding correction only.
- `git diff smd-official-import -- smd` and `git diff smd-official-import -- deps` remain empty. No SMD algorithm, planner, projection, ALM, checkpoint, architecture, dynamics, constraints, loss, weights, seed behavior, or data was changed.

### Inference and collision smoke

- Command: `conda run -n smd-blackwell python launch_smd_composite_experiment.py --start_index 0 --end_index 1 --map_name instances_simple` from `scripts/inference`.
- `SMOKE_STATUS=PASS_WITH_COMPAT_DELTA`: SDF and official trajectory dataset loaded (`n_trajs: 1000`, `trajectory_dim: (64, 12)`), model channels loaded, start/goal tensors printed with `device='cuda:0'`, official projection completed, and result output was written under `scripts/inference/results_test/2026-08-08-19-57-32/`.
- `CHECKPOINT_LOAD_STATUS=PASS`; `SMD_GPU_EXECUTION_CONFIRMED=true` based on the CUDA device of the key start/goal tensors and successful CUDA inference iterations.
- `is_collision.py` was run only after inference created `results_test/`. It is `COLLISION_STATUS=NOT_APPLICABLE_AT_S0_SMOKE`: the official checker is hard-coded for nine-agent result paths, while this authorized smoke is three-agent, and it encountered a missing `map_info.pkl` in a stale failed-run path. No collision-checker source was changed and no nine-agent benchmark was started.

### Storage and artifacts

- `FREE_DISK_SPACE_BEFORE_GB=199.67`; `FREE_DISK_SPACE_AFTER_GB=191.92`; `DISK_DELTA_GB=-7.75`.
- The existing 609486811-byte checkpoint archive and extracted data were reused; no duplicate checkpoint, clone, WSL2 distro, Docker image, CUDA Toolkit, video, or benchmark trajectory set was created.
- Evidence manifests: `experiments/manifests/system_manifest_s0_r1.txt`, `pip_freeze_s0_r1.txt`, `conda_list_s0_r1.txt`, `requirements_s0_blackwell_compat.txt`, and `gpu_smoke_s0_r1.txt`.
- This remediation remains limited to S0. No S1 benchmark, multi-map, multi-seed, 6/9-agent study, training, tuning, TR-SMD innovation, or paper experiment was started.

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

## S1-R0 Official SMD 3-Agent Functional Reproduction

- TASK_ID: `S1-R0-OFFICIAL-SMD-3AGENT-FUNCTIONAL-REPRODUCTION-R1`
- S0 closeout: `fix/s0-native-blackwell-compat` was fast-forwarded into `repro/smd-official`, then into `main`; both remote branches resolve to `caec78dddc2b1e04b3ad4385928d178a9076fd2e`. GitHub default branch was restored to `main`.
- S1 branch: `repro/s1-smd-functional`, starting at `main@caec78dddc2b1e04b3ad4385928d178a9076fd2e`.
- Official upstream head: `c87fc76044b350a37fcea7afc468c13c8371a237`; baseline tag: `smd-official-import`.

### S0 compatibility evidence versus S1 functional evidence

- S0 established the native Blackwell compatibility delta: Python 3.9.25, torch 2.7.1+cu128, torchvision 0.22.1+cu128, torchaudio 2.7.1+cu128, networkx 2.8.8, and RTX 5060 Ti capability `(12, 0)`. This remains explicitly distinct from the official Python 3.8/Torch 2.1.2 attempt.
- S1-R0 used the existing S0 `smd-blackwell` environment and did not download or modify checkpoint/data archives.
- `git diff smd-official-import -- smd` and `git diff smd-official-import -- deps` were empty. The entrypoint diff contained only the three previously approved `map_name` to `args.map_name` variable-binding fixes.

### Environment, data, and checkpoint gates

- `PYTHON=3.9.25`, `TORCH=2.7.1+cu128`, `TORCH_CUDA=12.8`, `GPU=NVIDIA GeForce RTX 5060 Ti`, `CAPABILITY=(12, 0)`, `CUDA_AVAILABLE=True`, `IPOPT_STATUS=True`.
- GPU tensor sanity test passed before inference.
- `instances_data/instances_simple.pkl` exists with 250 instances; `init4proj_data/instances_simple_init4proj_agent_3.pkl` exists with 250 entries; the three-agent checkpoint directory exists under `data_trained_models/EnvEmptyNoWait2D-RobotCompositeThreePlanarDisk/checkpoints/`.
- Existing `data_checkpoints.tar.gz` SHA-256 remains `5FB165686FA55E8955D842FA167B52ACD93A03AA513CD60787FEDF877B51689B`.

### Official 3-agent functional run

- Command: `conda run --no-capture-output -n smd-blackwell python launch_smd_composite_experiment.py --start_index 0 --end_index 1 --map_name instances_simple --save_path results_s1_r0_3agent` from `scripts/inference`.
- Fixed selection: `instances_simple`, instance index `0`, `EnvEmptyNoWait2DRobotCompositeThreePlanarDiskRandom`, 3 agents, `SMDComposite`, `SMDEnsemble`, official default planner/projection/checkpoint parameters.
- The log proves trajectory dataset loading (`n_trajs: 1000`, `trajectory_dim: (64, 12)`), model channel construction, CUDA start/goal tensors on `cuda:0`, projection at steps 15/5/0 with 29 ALM iterations at step 15, and result serialization. Runtime was `29.363261222839355` seconds.
- `paths.npy`: shape `(64, 64, 12)`, dtype `float32`, SHA-256 `76024E724A2F51E4891B6EEC602E2DA060DEAD504AC23BC5C89D6F9020BB53FB`, all finite.
- `map_info.pkl`: SHA-256 `0EB1AD3F2261B225F4DA75F94C3F78630587DCB286218F09EBC048D574AB2CD8`.
- Official output positions were parsed without editing raw data to `(3, 64, 2)`.

### Provisional collision and sanity checks

- `scripts/reproduction/check_smd_3agent_smoke.py` is a reproduction-only wrapper. It selects the three-agent map record and extracts the exact `is_collision.py::check_paths_ok` function body via AST so the official script's hard-coded nine-agent top-level scan is not executed. It does not alter paths, radius, threshold, or collision logic.
- Collision parameters were fixed at `robot_radius=0.05` and `threshold=1e-3`, matching the official checker.
- `COLLISION_FREE=True`; minimum inter-agent center distance was `0.18660226464271545 m`; provisional minimum obstacle clearance was `0.02442402451804533 m`.
- Start position errors per agent: `[5.960464477539063e-08, 0.0, 1.6858739410245458e-08] m`.
- Goal position errors per agent: `[4.915124922919302e-08, 2.665600744884954e-08, 2.384185793236071e-08] m`.

### Reproduction artifacts and scope

- Raw result directory: `scripts/inference/results_s1_r0_3agent/2026-08-08-20-09-42/`.
- Summary: `experiments/summaries/s1_r0_3agent_smoke_summary.json`.
- Evidence figure: `experiments/figures/s1_r0_smd_3agent_instances_simple_idx0.png`; SHA-256 `77CD17151AECDF34439A1D562EBB626A902486CA10D274EC79EC677DF87A580C`.
- Disk: `FREE_DISK_SPACE_BEFORE_GB=191.4`, `FREE_DISK_SPACE_AFTER_GB=191.15`, `DISK_DELTA_GB=-0.25`; no duplicate checkpoint/data, WSL2, Docker, CUDA Toolkit, benchmark, or extra agent run was created.
- This section is S1-R0 functional reproduction evidence only. It is not a paper metric pipeline, S1 closeout, multi-map benchmark, 6/9-agent run, tuning, or TR-SMD experiment.

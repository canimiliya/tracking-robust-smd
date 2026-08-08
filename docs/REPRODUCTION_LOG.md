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

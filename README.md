# gym-chrono

[QUICKSTART GUIDE](/docker/)

This repository consists of a set of gymnasium "environments" which are essentially wrappers around pychrono. In order to install gym-chrono, we must first install its dependecies
1) [pychrono](https://github.com/zzhou292/chrono/tree/feature/robot_model)
2) [gymnasium](https://pypi.org/project/gymnasium/)
3) [stable-baselines3[extra]](https://pypi.org/project/stable-baselines3/)

### The off-Road Gator Example Update

The `off_road_gator` example has been updated to use the **Chrono main repository**, which can be found at [https://github.com/projectchrono/chrono](https://github.com/projectchrono/chrono).

## ChronoViperWalk-v0 (Chrono 10 + Gymnasium)

This repository registers **`ChronoViperWalk-v0`**: a **headless** Gymnasium environment built on **Chrono 10** PyChrono bindings (`ChFramed`, `ChVector3d`, `SetGravitationalAcceleration`, NSC + **Bullet** collision) and **`pychrono.robot.Viper`**. Observation and action shapes are **18** and **12** respectively (the same *sizes* as the older `quadruped_walk` env for tooling compatibility). The reward is a simple **forward Δx** signal — it is **not** a drop-in replacement for every published quadruped benchmark.

**Prerequisites:** PyChrono with the **robot** module, `gymnasium`, `numpy`, and `CHRONO_DATA_DIR` pointing at a Chrono data tree (see below).

**Smoke test (from the repository root, after setting `PYTHONPATH`):**

```bash
export CHRONO_DATA_DIR="/path/to/chrono/data/"
export PYTHONPATH="/path/to/gym-chrono:$PYTHONPATH"
python3 gym_chrono/test/smoke_viper_walk.py
```

**One-liner check:**

```bash
python3 - <<'PY'
import gymnasium as gym
import gym_chrono  # registers envs
env = gym.make("ChronoViperWalk-v0", render_mode=None)
obs, _ = env.reset(seed=0)
obs2, r, term, trunc, _ = env.step(env.action_space.sample())
env.close()
print("obs", obs.shape, "->", obs2.shape, "r", r, "done", term or trunc)
PY
```

## Version matrix (reference)

Exact pins depend on your Chrono build and training stack. Use this table as a **documentation anchor**; adjust cells to match what you validate in CI.

| Component | Example pin / note |
|-----------|-------------------|
| **Project Chrono** / PyChrono | Build from [projectchrono/chrono](https://github.com/projectchrono/chrono) `main` (or your supported tag) with **Python** and **robot** enabled for `ChronoViperWalk-v0`. |
| **Python** | 3.10–3.12 (match your PyChrono wheel or build). |
| **gymnasium** | ≥ 0.29 (tested with modern `gym.make` API). |
| **numpy** | Match PyChrono’s supported NumPy line (many Chrono Python builds expect **1.24.x**). |
| **rsl-rl-lib** (optional) | If you use RSL-RL, align the **major** line with your YAML / config schema (2.x vs 5.x differ). |
| **PyTorch** (optional learner) | CUDA build **or** ROCm wheel from [PyTorch ROCm install](https://pytorch.org/get-started/locally/) — must match your driver stack. |

## Ray + PyTorch ROCm (CPU Chrono workers, GPU learner)

When you run **many CPU-only Chrono** simulations under **Ray** while a **PyTorch ROCm** policy trains on AMD GPUs, use two patterns:

1. **Set Ray’s ROCm-safe environment variables before the first `import ray`** (Ray ≥ 2.45 on PyTorch ROCm):

```python
import os

os.environ.setdefault("RAY_EXPERIMENTAL_NOSET_HIP_VISIBLE_DEVICES", "1")
os.environ.setdefault("RAY_EXPERIMENTAL_NOSET_ROCR_VISIBLE_DEVICES", "1")

import ray  # noqa: E402
```

The [ChronoRay](https://github.com/uwsbel/chrono-ray) package documents the same bootstrap under `docs/AMD_ROCM.md` and exposes `prepare_rocm_ray_env()` if you already depend on that project.

2. **Hide GPUs inside Ray workers** that only run CPU physics, so they do not attach to HIP devices:

```python
_CHRONO_CPU_ENV = {
    "env_vars": {
        "HIP_VISIBLE_DEVICES": "-1",
        "ROCR_VISIBLE_DEVICES": "-1",
    }
}


@ray.remote(num_cpus=1, runtime_env=_CHRONO_CPU_ENV)
class ChronoSimActor:
    def __init__(self, seed: int):
        import gymnasium as gym
        import gym_chrono  # noqa: F401

        self._env = gym.make("ChronoViperWalk-v0", render_mode=None)
        self._env.reset(seed=seed)

    def step(self, action):
        return self._env.step(action)
```

**AMD GPU selection** for the **learner** process should use **`ROCR_VISIBLE_DEVICES`**, not `CUDA_VISIBLE_DEVICES`. When starting Ray manually, pass **`ray start --num-gpus=N`** (with `N` matching visible devices); otherwise Ray may schedule **zero** GPUs even when hardware is present.

### Reviewer reproduction (generic AMD Linux: CPUs + Instinct)

1. Install **ROCm** and build **PyChrono** with **Python + robot**; set **`CHRONO_DATA_DIR`** and prepend the Chrono Python path to **`PYTHONPATH`** (see [Chrono Python install](https://api.projectchrono.org/module_python_installation.html)).
2. Clone this branch, prepend **`PYTHONPATH=$PWD`**, create a venv, `pip install gymnasium numpy`, run **`python3 gym_chrono/test/smoke_viper_walk.py`**.
3. For **Ray + Instinct**: export **`ROCR_VISIBLE_DEVICES`**, set **`RAY_EXPERIMENTAL_NOSET_HIP_VISIBLE_DEVICES`** and **`RAY_EXPERIMENTAL_NOSET_ROCR_VISIBLE_DEVICES`** before `import ray`, run **`ray start --head --num-gpus=N`**, then run **`python3 playground/reviewer_ray_chrono_smoke.py`** (minimal two-actor smoke; requires `ray` installed).

Full step-by-step reproduction (shell + inline Python) is in the **“Reviewer reproduction”** section of **[PR #19](https://github.com/projectchrono/gym-chrono/pull/19)** on GitHub.

## Downloading data files
Before you begin the installation process, you will need to download the `data` folder containing the simulation assets and place it in the right place:
1) Download the data files [here](https://drive.google.com/drive/folders/1u4nwAlpPXtgkSJeBLlSM9B_utEoUIY41?usp=drive_link), unzip if necessary, you should obtain a folder named `data`.
2) Copy the data to `DIR_OF_REPO/gym-chrono/envs`.

#### Adding Chrono data directory to path
Once the data folder has been downloaded and placed in the right folder, it needs to be added to path:  
For Linux or Mac users:  
  Replace bashrc with the shell your using. Could be `.zshrc`.  
  1. echo `export CHRONO_DATA_DIR=<Downloaded data directory path>' >> ~/.bashrc`  
      Ex. If you have cloned the repository in `home` , then, echo `export CHRONO_DATA_DIR=/home/user/gym-chrono/gym-chrono/envs/data/' >> ~/.bashrc`  
  2. `source ~/.bashrc`

For Windows users:  
  Link as reference: https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/  
  1. Open the System Properties dialog, click on Advanced and then Environment Variables  
  2. Under User variables, click New... and create a variable as described below  
      Variable name: CHRONO_DATA_DIR  
      Variable value: <chrono's data directory>  
          Ex. Variable value: C:\ Users\ user\ chrono\ data\

## Installing dependencies
### Installing pychrono
1) First you need to install pychrono from source. The Chrono source that needs to be cloned is linked [here](https://github.com/zzhou292/chrono/tree/feature/robot_model). Please use the feature/robot_model branch. We use this fork with this branch because it contains all the latest robot models that are not currently available in Chrono main.
2) Once you have the source cloned, build pychrono from source using the official [Python module installation](https://api.projectchrono.org/module_python_installation.html) guide. Enable modules Chrono::Sensor, Chrono::Irrlicht, Chrono::SynChrono, Chrono::Vehicle, Chrono::Python, Chrono::OPENMP and Chrono::Parsers. For each of these modules, please look at the official Chrono documentation.
3) Make sure you add the appropriate numpy include directory (see linked instructions above)
4) If you are not doing a system wide install of pychrono, make sure you add to PYTHONPATH the path to the installed python libraries (see linked instructions above)
### Installing gymnasium
```bash
pip install gymnasium
```
> [!NOTE]
> If you are using a conda environment, activate the conda environment and then use the same command above.  

### Installing stable-baselines3
```bash
pip install stable-baselines3[extra] 
```

> [!NOTE]
> `stable-baselines3` installs nupmy as a dependency, so it is recomended to remove this installation and install your own version of numpy. Additionally, `pychrono` requires `numpy=1.24.0`, and it must be installed with conda, so it is necessary to run `pip uninstall numpy` and `conda install -c conda-forge numpy=1.24.0` to not get a `pychrono.sensor` error.
### Rough Edges
#### Adding gym-chrono to path
Due to the lack of a pip installer for this package currently, you must add gym-chrono to `PYTHONPATH`:
```
 echo 'export PYTHONPATH=$PYTHONPATH:<path to gym-chrono>' >> ~/.bashrc
```
Replace `~/.bashrc` with `~/.zshrc` in case you are using `zsh`.<br>
For Windows users, follow instructions from [here](https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/).

     
## Repository Structure

This repository is structured as follows:
1. Within the `gym-chrono` folder is all that you need:
   - **env**: gymnasium environment wrapper to enable RL training using PyChrono simulation
   - **test**: testing scripts to visualize the training environment and debug it
   - **train**: python scripts to train the models for each example env with stable-baselines3
   - **evaluate**: python scripts to evaluate a trained model
2. The `playground` folder contains scripts that do not use Chrono as a simulation engine. This folder is maintained just for experimentation
3. The `images` folder consists of images used in the `readme` like the one below!

#### Here is a video of the example Gator environment on SCM deformable terrain with an 80 x 45 camera simulated with Chrono::Sensor   
![Gator demo](https://github.com/projectchrono/gym-chrono/blob/master/images/gator.gif)

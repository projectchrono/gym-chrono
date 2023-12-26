# gym-chrono
This repository consists of a set of gymnasium "environments" which are essentially wrappers around pychrono. In order to install gym-chrono, we must first install its dependecies
1) pychrono
2) gymnasium
3) stable-baselines3[extra]
4) opencv

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
1) First you need to install pychrono from source. The Chrono source that needs to be cloned is linked [here]([url](https://github.com/zzhou292/chrono/tree/feature/robot_model)https://github.com/zzhou292/chrono/tree/feature/robot_model). Please use the feature/robot_model branch. We use this fork with this branch because it contains all the latest robot models that are not currently available in Chrono main.
2) Once you have the source cloned, build pychrono from source using instructions found [here]([url](https://api.projectchrono.org/module_python_installation.html)https://api.projectchrono.org/module_python_installation.html). Enable modules Chrono::Sensor, Chrono::Irrlicht, Chrono::SynChrono, Chrono::Vehicle, Chrono::Python, Chrono::OPENMP and Chrono::Parsers. For each of these modules, please look at the official Chrono documentation.
3) Make sure you add the appropriate numpy include directory (see linked instructions above)
4) If you are not doing a system wide install of pychrono, make sure you add to PYTHONPATH the path to the installed python libraries (see linked instructions above)
### Installing gymnasium
```bash
pip install gymnasium
```
If you are using a conda environment, activate the conda environment and then use the same command above.  
Note: Conda and pip have separate mechanisms for managing dependencies. While Conda can see and manage the packages installed by pip, pip does not have visibility into the packages managed by Conda. This can sometimes lead to dependency conflicts or issues if a package installed via pip requires a different version of a dependency than what is already installed in the Conda environment by Conda. Since `gymnasium` and `stable-baselines3` do not have conda installers, we recommend using only `pip` even within the conda environment.

### Installing stable-baselines3
```bash
pip install stable-baselines3[extra] 
```

### Installing opencv
`opencv` is used to generate random terrain height maps in the form of bitmaps. To install use:
```bash
pip install opencv-python 
```

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

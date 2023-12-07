# gym-chrono
This repository consists of a set of gymnasium "environments" which are essentially wrappers around pychrono. In order to install gym-chrono, we must first install its dependecies
1) pychrono
2) gymnasium
3) stable-baselines3[extra]

NOTE: the current gym-chrono is pegged to chrono fork: https://github.com/zzhou292/chrono, 'feature/robot_model' branch.

## Repository Structure

This repository is structured as follows:

- **env**: gymnasium environment wrapper to enable RL training using PyChrono simulation
- **test**: testing scripts to visualize the training environment and debug it
- **training**: python scripts to train the models for each example env
- **evaluate**: python scripts to evaluate a trained model


Currently, this repository also contains scripts to train simple RL models using stable-baselines3 (see gym_chrono/training). So, you would also need to install stable-baselines3
The setup has a few rough edges which will be evened out in due time.  

## Downloading data files
1) Download the data files [here](https://drive.google.com/drive/folders/1u4nwAlpPXtgkSJeBLlSM9B_utEoUIY41?usp=drive_link), unzip if necessary, you should obtain a folder named "data".
2) Copy the data to DIR_OF_REPO/gym-chrono/envs.
   
## Installing dependencies
### Installing pychrono
1) First you need to install pychrono from source. The Chrono source that needs to be cloned (for now) is linked [here]([url](https://github.com/zzhou292/chrono/tree/feature/robot_model)https://github.com/zzhou292/chrono/tree/feature/robot_model). Please use the feature/robot_model branch. We use this fork with this branch because it contains all the latest robot models that are not currently available in Chrono main.
2) Once you have the source cloned, build pychrono from source using instructions found [here]([url](https://api.projectchrono.org/module_python_installation.html)https://api.projectchrono.org/module_python_installation.html). Enable modules Chrono::Sensor, Chrono::Irrlicht, Chrono::SynChrono, Chrono::Vehicle, Chrono::Python, Chrono::OPENMP and Chrono::Parsers. For each of these modules, please look at the official Chrono documentation.
3) Make sure you add the appropriate numpy include directory (see linked instructions above)
4) If you are not doing a system wide install of pychrono, make sure you add to PYTHONPATH the path to the installed python libraries (see linked instructions above)
### Installing gymnasium
```
pip install gymnasium
```
If you are using a conda environment, install pip in the conda environment and install gymnasium using the pip from the conda environment. For example
```
/home/USER/anaconda3/envs/ENV_NAME/bin/pip3 install gymnasium
```
### Installing stable-baselines3
```
pip install stable-baselines3[extra] 
```
### Rough Edges
#### Adding gym-chrono to path
Due to the lack of a pip installer for this package currently, you must add gym-chrono to PYTHONPATH. For example, you could do something like
```
 echo 'export PYTHONPATH=$PYTHONPATH:<path to gym-chrono>' >> ~/.bashrc
```
Replace `~/.bashrc` with `~/.zshrc` in case you are using `zsh`.<br>
For Windows users, follow instructions from [here](https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/).

#### Adding Chrono data directory to path
Since the environments use data files from chrono's build directory, the data folder in the chrono build directory needs to be added to path.  
For Linux or Mac users:  
  Replace bashrc with the shell your using. Could be .zshrc.  
  1. echo 'export CHRONO_DATA_DIR=<chrono's data directory>' >> ~/.bashrc  
      Ex. echo 'export CHRONO_DATA_DIR=/home/user/chrono/data/' >> ~/.zshrc  
  2. source ~/.bashrc  

For Windows users:  
  Link as reference: https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/  
  1. Open the System Properties dialog, click on Advanced and then Environment Variables  
  2. Under User variables, click New... and create a variable as described below  
      Variable name: CHRONO_DATA_DIR  
      Variable value: <chrono's data directory>  
          Ex. Variable value: C:\ Users\ user\ chrono\ data\  
#### More data
To make matters worse, there is some more data downloaded.  
Install the data folder from [here](https://uwmadison.box.com/s/4hhb9ldtzsih5kcupf7ttsjr8t8hs550), and place it `gym_chrono/envs`.

#### Here is a video of the example Gator environment on SCM deformable terrain with an 80 x 45 camera simulated with Chrono::Sensor   
![Gator demo](https://github.com/projectchrono/gym-chrono/blob/master/images/gator.gif)
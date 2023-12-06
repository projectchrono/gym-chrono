# gym-chronoV2
This repository consists of a set of gymnasium "environments" which are essentially wrappers around pychrono. In order to install gym-chrono, we must first install its dependecies
1) pychrono
2) gymnasium

NOTE: the current gym-chrono is pegged to chrono fork: https://github.com/zzhou292/chrono, 'feature/robot_model' branch.

## Repository Structure

This repository is structured as follows:

- **env**: gymnasium environment wrapper to enable RL training using PyChrono simulation
- **test**: testing scripts to visualize the training environment
- **training**: python scripts to train the models for each example env
- **evaluate**: python scripts to evaluate a trained model


Currently, this repository also contains scripts to train simple RL models using stable-baselines3 (see gym_chrono/training). So, you would also need to install stable-baselines3
The setup has a few rough edges which will be evened out in due time.  
## Installing dependencies
### Installing pychrono
1) First you need to install pychrono from source. The Chrono source that needs to be cloned (for now) is linked [here]([url](https://github.com/zzhou292/chrono/tree/feature/robot_model)https://github.com/zzhou292/chrono/tree/feature/robot_model). Please use the feature/robot_model branch.
2) Once you have the source cloned, build pychrono from source using instructions found [here]([url](https://api.projectchrono.org/module_python_installation.html)https://api.projectchrono.org/module_python_installation.html). Enable modules Chrono::Sensor, Chrono::Irrlicht, Chrono::SynChrono, Chrono::Vehicle, Chrono::Python and Chrono::OPENMP.
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
 echo 'export PYTHONPATH=$PYTHONPATH:<path to gym-chronoV2>' >> ~/.bashrc
```
Replace `~/.bashrc` with `~/.zshrc` in case you are using `zsh`.<br>
For Windows users, follow instructions from [here](https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/).

#### Adding Chrono data directory to path
Since the environments use data files from chrono's build directory, the data folder in the chrono build directory needs to be added to path.  
For Linux or Mac users:  
  Replace bashrc with the shell your using. Could be .zshrc.  
  1. echo 'export CHRONO_DATA_DIR=<chrono's data directory>' >> ~/.bashrc  
      Ex. echo 'export CHRONO_DATA_DIR=/home/user/chrono/data/' >> ~/.zshrc  
  2. source ~/.zshrc  

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
  

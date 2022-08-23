# ALIVE_work
Some of the code I developed during my tenure at ALIVE

A sample readme I made during my tenure:

# Running PRSG path-planning DEMO on CARLA
Assuming the user has successfully compiled all the nodes using catkin_make or cmake. They can follow these steps, 
``` sh
$ cd <carla-repo-path>
$ ./carlaUE4.sh
```
open a new tab
```sh
$ source /opt/carla_ros_bridge/melodic/setup.bash
$ roslaunch carla_ros_bridge carla_ros_bridge.launch
```
Now to interface the Carla server with your code/manually control the car open a new tab
```sh 
$ cd <carla-repo-path>
$ python PythonAPI/examples/

```
Here the name of the environment is called **commonroad-py37**. You may also change this name as you wish. In such case, don't forget to change it in the following commands as well. **Always activate** this environment before you do anything related:
```sh
$ conda activate commonroad-py37
or
$ source activate commonroad-py37
```
### Commonroad libraries installation:
#### 1. Commonroad-io
```sh
$ pip install commonroad-io
```
#### 2. Commonroad-sumo-interface
Install required packages:
```sh
cd commonroad-sumo-interface
pip install -r requirements.txt
sudo apt-get install ffmpeg
```
#### 3. Commonroad-search
To install the required dependencies, first change your base folder path inside run.sh then run the following commands:
```sh
cd ..
chmod +x run.sh
./run.sh
```
And add the python path of `commonroad-sumo-interface` to your Python interpreter.
## 3. Running commonroad motion planner
Add this line to your ~/.bashrc
export SUMO_HOME="/usr/share/sumo"
export PATH="/usr/share/sumo/tools:$PATH"
Run the OSM web wizard file to choose a geographic area for creating a map(using gui) by executing
```sh
python /usr/share/sumo/tools/osmWebWizard.py
```
#### NOTE: This creates a dated folder in your current directory.
Edit your pathconfig_Default.py with appropriate python path and save the file as pathconfig.py
Change the name of this folder as per your understanding (say x).
Run these commands after replacing the net_file path and sumo_cfg_file path inside the `create_scenario_files.py`(inside commonroad-sumo-interface/example_scripts) to generate openDRIVE(.xodr) and lanelet(.cr.xml) files in one go.
```sh
cd /path/to/your/cloned-repo/commonroad-sumo-interface/example_scripts
python create_scenario_files.py
```
#### NOTE: Potential error: If the above command shows "Scenario_id not found/ required" error, please make the following change to ~/miniconda3/envs/commonroad-py37/lib/python3.7/site-packages/opendrive2lanelet/network.py
in export_commonroad_scenario function
```py
scenario = scenario(dt=dt, scenario_id=None, benchmark_id if benchmark_id is not None else "none")
```
This creates a folder named 'osm' in the scenarios directory of commonroad-sumo-interface. Change the name of this folder to (x).
Next we create a path planning problem inside the .cr.xml file present in the (x) folder.
```sh
cd /path/to/your/cloned-repo/scripts
python pps_adder.py
```
#### NOTE: Add python path of commonroad-search folder to ~/.bashrc
Now the solution to the motion planning problem can be found by executing, before executing change the 'paths_scenario' inside the scripts/Trajectory_planning.py
```sh
python Trajectory_planning.py
```


# Carla installation
If you want to run carla with pygame use the Quick Start installation. In most cases, we will use this installation because we do not need to edit the maps.
### Quick Start Installation
Install required dependencies.
```sh
pip install --user pygame numpy
```
Download the .tar.gz file and additional maps for linux for carla version 0.9.9.2 from the following link. We use an old version because at the time of this documentation, this version was compatible with external opendrive files.
```sh
https://github.com/carla-simulator/carla/releases
```
Extract Carla_version.tar.gz at some location and then copy paste the AdditionalMaps.varsion.tar.gz file in the Imports folder. Then run the following commands to download additional maps.
```sh
cd /path_to_your_carla_directory
./ImportAssets.sh
```
Add the pythonpath to the egg file and carla folder in your bashrc.
```sh
export PYTHONPATH=${PYTHONPATH}:~/path_to_your _folder/CARLA_0.9.9.2/PythonAPI/carla/dist/carla-0.9.9-py3.7-linux-x86_64.egg
export PYTHONPATH=${PYTHONPATH}:~/path_to_your _folder/CARLA_0.9.9.2/PythonAPI/carla
```
Run carla
```sh
./CarlaUE4.sh
```
Run a script in a new terminal to test CARLA.
```sh
cd PythonAPI/examples
python3 spawn_npc.py
```
If you want to run carla with unreal engine use the following installation.
### Carla Build with UE4 Installation
##### Note: This is a heavy installation. Use it on a systems with at least 16 GB RAM and a dedicated graphic card. Also the complete installation takes up about 90 GB of space. 
&nbsp;
Install the required dependencies.
```sh
sudo apt-get update &&
sudo apt-get install wget software-properties-common &&
sudo add-apt-repository ppa:ubuntu-toolchain-r/test &&
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add - &&
sudo apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-8 main" &&
sudo apt-get update
```
Additional dependencies for Ubuntu 18.04.
```sh
sudo apt-get install build-essential clang-8 lld-8 g++-7 cmake ninja-build libvulkan1 python python-pip python-dev python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev &&
pip2 install --user setuptools &&
pip3 install --user -Iv setuptools==47.3.1 &&
pip2 install --user distro &&
pip3 install --user distro
```
To avoid compatibility issues between UE4 and Carla, change the default clang version.
```sh
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-8/bin/clang++ 180 &&
sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-8/bin/clang 180
```
#### 1. Installing UE4
Get a GitHub and a UE account, and link both. 
Install git. 
##### Note: CARLA will need the UE 4.24 release, not the latest.
Download Unreal Engine 4.24.
```sh
git clone --depth=1 -b 4.24 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.24
cd ~/UnrealEngine_4.24
```
Download and install the UE patch with the following commands. This is to remove any vulcan errors that might come.
```sh
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/UE_Patch/430667-13636743-patch.txt 430667-13636743-patch.txt
patch --strip=4 < 430667-13636743-patch.txt
```
Make the build. This may take an hour or two depending on your system.
```sh
./Setup.sh && ./GenerateProjectFiles.sh && make
```
Open the Editor to check that UE has been properly installed. First time opening the UE editor will take a lot of time because it installs the shaders and other stuff related to the maps.
```sh
cd ~/UnrealEngine_4.24/Engine/Binaries/Linux && ./UE4Editor
```
#### 2. Installing Carla
Downloading aria2 speed up the subsequent commands.
```sh
sudo apt-get install aria2
```
Clone the CARLA repository and Get the CARLA assets
```sh
git clone https://github.com/carla-simulator/carla
cd ~/carla
./Update.sh
```
Add the following command to bashrc and source it. This command is required so that carla can find the UE4 installation folder.
```sh
export UE4_ROOT=~/UnrealEngine_4.24
source ~/.bashrc
```
Make the CARLA client and the CARLA server
```sh
make PythonAPI
make launch
```
Press play in the Editor to initialize the server
Run example scripts to test CARLA
Terminal A 
```sh
cd PythonAPI/examples
python3 spawn_npc.py 
```
Terminal B
```sh
cd PythonAPI/examples
python3 dynamic_weather.py
```
# Carla ROS bridge interface

## Carla ROS bridge installation
Please make sure you've disabled all conda environments, and have installed ros-melodic. Also, make sure the command ```python -V``` returns Python 2.7.X. If it returns some Python 3 version make sure that Python 2.7 is [symbolically linked](https://stackoverflow.com/questions/22626867/howto-symlink-python-to-python2-7-as-non-superuser).
### Installation
First add the apt repository:
```sh
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
```
Then simply install the ROS bridge:
```sh
sudo apt-get update
sudo apt-get install carla-ros-bridge
```
This will install carla-ros-bridge in ```/opt/carla-ros-bridge```
Add python egg file from Carla 0.9.9.2 folder to ~/.bashrc file.
```sh
export PYTHONPATH=$PYTHONPATH:<path-to-carla>/PythonAPI/carla/dist/carla-0.9.9-py2.7-linux-x86_64.egg
source ~/.bashrc
```
also add the following command to bashrc file.
```sh
source /opt/carla-ros-bridge/<kinetic or melodic or noetic>/setup.bash
```
### Running an example
To run a sample launch file open a new terminal window and run:
```sh
./<path-to-your-carla-0.9.9.2>/CarlaUE4.sh -opengl4
```
or to run with vulkan drivers use,
```sh
./<path-to-your-carla-0.9.9.2>/CarlaUE4.sh
```
Next in a new terminal(conda disabled) run 
```sh
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```
### Basic Troubleshooting
1. If the command shows no module carla found, please make sure you've sourced the python path to the egg file correctly.
2. If it shows  ```undefined symbol: PyString_Type``` then make sure your Python 2.7 is symbolically linked, you've sourced the Python 2.7 egg file.
3. If it shows missing libraries, please install accordingly for python 2.7 using pip command.
### Developement
For writing your own ros nodes you can enable your conda environment and use python3 command for execution. 
Next to test this you can execute the following commands:
```sh
cd /<path-to-your-carla-0.9.9.2>/PythonAPI/examples
python manual_control.py
```
in a new terminal window
```sh 
cd /<path-to-this-repo>/scripts
python manual_twist.py
```
You can use twist rostopic to control the ego vehicle.



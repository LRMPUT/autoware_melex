# AUTOWARE_MELEX

## Contribution
1. AutowareAuto repo and 3rd party packages have to stay unchanged.
2. New packages:
    * Same paths as AutowareAuto
    * Same names as AutowareAuto with ```melex``` prefix

    Example: 

    * AutowareAuto package path: **~/AutowareAuto/src/mapping/ndt_mapping_nodes/launch/ndt_mapper.launch.py**
    
    * Melex package path: **~/AutowareAuto/src**/autoware_melex/**mapping**/melex_**ndt_mapping_nodes/launch/ndt_mapper.launch.py**
    

3. If new package doesn't have corresponding path within AutowareAuto repo due to totally new functionality, try use one of existing directory (e.g. drivers, mapping etc.)
4. 3rd party repos has to be cloned into ```external``` directory using vcs tool. You just need to update ```autoware_melex.repos``` file.
5. Packages should contain README.md file at least with three sections: description, todo, known issues. 

Creating a new package even for a single parameter file seems a bit strange now. But as the package grows, it will be easier to work with ```autoware_melex``` repo if it is mirrored with ```AutowareAuto``` package.    

## Info
1. With .aderc-development config is necessary to run simulator with flag
```
/opt/lgsvl/simulator --hostname 172.30.0.100
```

## Installation
1. [Install ADE](https://ade-cli.readthedocs.io/en/latest/install.html).
2. Setup ADE home and clone AutowareAuto repo
```
mkdir -p ~/adehome
cd ~/adehome
touch .adehome
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
```
3. Clone autoware_melex repo
```
cd ~/adehome/AutowareAuto/src
git clone https://github.com/LRMPUT/autoware_melex
```
4. Set a new docker network (required by .aderc-development config)
```
docker network create ade_network --subnet=172.30.0.0/24
```
5. Build a development image (required by .aderc-development config)
```
cd ~/adehome/AutowareAuto/src/autoware_melex/docker
./build.sh
```
7. Start ade with config
```
cd ~/adehome/AutowareAuto/src/autoware_melex
```
* Option 1: on Melex:
```
ade --rc .aderc-melex  start  --update --enter
```
* Option 2: on custom host:
```
ade --rc .aderc-development  start  --update --enter
```

8. Import repos
```
cd AutowareAuto
vcs import < autoware.auto.$ROS_DISTRO.repos
cd src/autoware_melex
vcs import < autoware_melex.repos
```
9. Build workspace
```
cd ~/AutowareAuto
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
10. Load environment variables
```
source ~/AutowareAuto/install/setup.bash
```

## IDE configuration
Setup IDEs using SSH with credentials (works with .aderc-development config):
   * Address: 172.30.0.100
   * Port: 22
   * Login: ade
   * Password: ade 
     
### Python 
   Use remote interpreter path: ```/usr/bin/remote_python3``` (works with .aderc-development config)
### C++     
   To get environments variables for CMake configuration, run ```./env_vars.sh``` and use script output.

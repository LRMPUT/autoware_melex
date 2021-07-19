# AUTOWARE_MELEX

## Contribution
1. AutowareAuto repo and 3rd party packages have to stay unchanged.
2. New packages:
    * Same paths as AutowareAuto
    * Same names as AutowareAuto with ```melex``` prefix

    Example:
    AutowareAuto contains mapping functionality. For our purpose it was necessary to modify some parameters within launch file and configs.
   
    AutowareAuto path: <span style="color: green">mapping/ndt_mapping_nodes/launch/ndt_mapper.launch.py</span>
    
    Melex path: <span style="color: green">mapping/</span>melex_<span style="color: green">ndt_mapping_nodes/launch/ndt_mapper.launch.py</span>
    
    Melex path cloned into AutowareAuto: autoware_melex<span style="color: green">/mapping/</span>melex_<span style="color: green">ndt_mapping_nodes/launch/ndt_mapper.launch.py</span>

3. If new package doesn't have corresponding path within AutowareAuto repo due to totally new functionality, try use one of existing directory (e.g. drivers, mapping etc.)
4. 3rd party repos has to be cloned into ```external``` directory using vcs tool. You just need to update ```autoware_melex.repos``` file.
5. Packages should contain README.md file at least with three sections: description, todo, known issues. 

Creating a new package even for a single parameter file seems a bit strange now. But as the package grows, it will be easier to work with ```autoware_melex``` repo if it is mirrored with ```AutowareAuto``` package.    

## ToDo
* Create a shared space in PUT cloud for storing large media files (e.g. rosbags). Stock 2 GB is not enough.


## Info
1. With modified ADE config is necessary to run simulator with flag
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
git clone https://gitlab.com/amadeuszsz/autoware_melex.git
```
4. Set a new docker network
```
docker network create ade_network --subnet=172.30.0.0/24
```
5. Get an image
```
docker pull amadeuszsz/ade-foxy:development
```
7. Start ade with config
```
cd autoware_melex
ade --rc .aderc-melex  start  --enter
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
colcon build
```

## IDE configuration
Setup IDEs using SSH with credentials:
   * Address: 172.30.0.100
   * Port: 22
   * Login: ade
   * Password: ade 
     
### Python 
   Use remote interpreter path: ```/usr/bin/remote_python3```
### C++     
   To get environments variables for CMake configuration, run ```./env_vars.sh``` and use script output.

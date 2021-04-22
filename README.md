# ROS 2 Person Detection #

This project implements a human detection by an artificial neural network based on ROS 2.  
The available detectors are

* [Person-Detection](https://github.com/SusmithKrishnan/person-detection) by [SusmithKrishnan](https://github.com/SusmithKrishnan)
* [Detectron2](https://github.com/facebookresearch/detectron2) by [Facebook](https://github.com/facebookresearch)

*__Notice:__ All data is included in this repository and the installation of those networks is executed during the following instructions.*  
*__Notice:__ For the online person detection a cognex camera is required but it is also possible to use the detection part on offline images and videos.*

## Requirements ##

Ubuntu 20.04, Python 3.8+  
Please make sure that you have installed all the required components before building the project.

### ROS 2 Foxy ###

* Go [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/) for installation of ROS 2 Foxy. Make sure to install the ros-foxy-desktop package.  
* If this is the first distribution of ROS that you installed on your PC, please execute (if not already done in the ROS Foxy installation):  
`echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc`  
* If you already installed a ROS distribution on your PC, go to your home directory and check the file ".bashrc" for the line  
`"source /opt/ros/.../setup.bash"`  
and replace the distribution with "foxy". If you can't see the file ".bashrc" in your home directory press "Strg + H".


### General Packages ###

* For using rosdep:  
`sudo apt-get install python3-rosdep`
* For using colcon:  
`sudo apt-get install python3-colcon-common-extensions`
* For the different detectors:  
`sudo apt-get install python3-venv`
* For the usage of git:  
`sudo apt-get install git`
* For linking the env to python3:  
`sudo apt-get install python-is-python3`


## Setting up the Project ##

1. Create new development workspace  
`mkdir -p ~/dev_ws/src`
2. Go to the source folder of your workspace  
`cd ~/dev_ws/src`
3. Clone the repository  
`git clone <link_to_repo>`
4. Go to models  
`cd ros2_person_detection/detection_driver/models/`
5. Create a virtual environment  
`python3 -m venv ~/venv`
6. Enter the virtual environment  
`source ~/venv/bin/activate`
7. Install pip wheels  
`pip3 install wheel`
8. Install requirements  
`pip3 install -r requirements.txt`
9. Install Detectron2  
`pip3 install -e detectron2`
10. Exit from virtual environment  
`deactivate`
11. Go back to top level  
`cd ~/dev_ws`
12. Build the project  
`colcon build --symlink-install`


## How to Run the Project ##

1. Check the configurations inside the packages  
`package_name -> configs`
2. If config was changed make sure to rebuild. You can avoid this by copying a config file to your home directory and use the param_file launch parameter (equivalent to venv) to refer to this config file.  
`cd ~/dev_ws && colcon build --symlink-install`
3. Source the workspace  
`source ~/dev_ws/install/setup.bash`
4. Run the Desired Programm:  

    Available Launch Parameters:
    
    * venv -> absolute path to the python interpreter of your virtual environment
    * param_file -> absolute path to the config file that should be used
    
    **Attention**:
    
    * Make sure to replace /path/to/ in the venv launch parameter with the absolute path to your virutal environment python interpreter.  
    * The venv parameter for the Cognex driver is only required if OpenCV is not installed on your system.

    Programm                 | Requirements                                                                                               | Command
    ---                      | ---                                                                                                        | ---
    Person Detection         | Connected Cognex camera, Check the config file `person_detection/configs/default.yaml`, Rebuild if necessary | `ros2 launch person_detection person_detection.launch.py venv:=/path/to/venv/bin/python`
    Person Detection Testing | No Cognex is required, Set the test_data parameter in `person_detection/configs/test.yaml`, Rebuild          | `ros2 launch person_detection person_detection_test.launch.py venv:=/path/to/venv/bin/python`
    Cognex Camera Driver     | Connected Cognex camera, Check the config file `cognex_driver/configs/default.yaml`, Rebuild if necessary    | `ros2 launch cognex_driver cognex_driver.launch.py venv:=/path/to/venv/bin/python`
    Detection Driver         | No Cognex required, Check the config file `detection_driver/configs/default.yaml`, Rebuild if necessary         | `ros2 launch detection_driver detection_driver.launch.py venv:=/path/to/venv/bin/python`


## Trouble Shooting ##

* If there are errors about missing modules like cv_bridge or similar. Make sure that you installed the ros-foxy-desktop and not the ros-foxy-base.
* If there are errors about missing modules of the detetction_driver you might have forgot the venv parameter at the launch command.

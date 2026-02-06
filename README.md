# GeiCar Project - *Hold My Wheel*

The GeiCar project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous car in order to carry out different missions. Several projects are exposed on the [official website](https://sites.google.com/site/projetsecinsa/).

*Hold My Wheel* is a driving assistance focused project that integrates modern vehicle features such as obstacle detection and avoidance, airbag deployment or lane centering assist. These driving assistance features are often only found in high-end vehicles, but our project aims to deliver driving safety in an accessible, low-cost and open-source solution.

For additional documentation, see :
* The [group drive](https://drive.google.com/drive/folders/1c30rR2X9cSf7GfaXyJIrqkAkz-WrVF_N?usp=sharing)
* The [project plan](https://docs.google.com/document/d/17D9ZPcc3gT9gmjuxm5dIHnuOJmiaQUO5/edit?usp=sharing&ouid=109228807280577267914&rtpof=true&sd=true)
* The [project architecture notes](https://docs.google.com/document/d/1LTkkd4-w4RaJ8ROUtFZ62aQJ4dhyVaZO-zE2RoP_Y3U/edit?usp=sharing)
* The [project site](https://sites.google.com/view/insa-5siec/projets-2025-2026/kleya-project?authuser=0)

This repository is a forked of a basis for students starting a new project on the GeiCar. The forked code as well as the documentation is the result of internship carried out by [Alexis24](https://github.com/Alexix24) (Alexis Pierre Dit Lambert)

The platform is developped and maintained by :

* DI MERCURIO Sébastien
* LOMBARD Emmanuel
* MARTIN José

The projects are (or were) surpervised by:

* CHANTHERY Elodie
* DELAUTIER Sébastien
* MONTEIL Thierry
* LE BOTLAN Didier
* AURIOL Guillaume
* DI MERCURIO Sébastien

## Quick User Guide
### Turn the car on and off
* To turn on the car:
  * Toggle the red button to bring the power.
  * Press the START push button (hold it down for a short while).
  * Wait until Raspberry boot up and connect to it using its IP address (written on the board): `ssh pi@10.105.1.xx`
  * When connected start ROS subsystem using : `source ~/test/develop/geicar-kleya/raspberryPI3/ros2_ws/launch.sh`
  * Then, you will get a report of subsystems and be able to control the car using XBOX controller 
  * For the LCA system, connect to the jetson Nano : `ssh jetson@192.168.1.xx` (the IP can change, see Project Architecture Notes)
  * Start the docker : `sudo docker start -ai ros-humble`
  * Launch the jetson ROS subsystem : `source /root/test/develop/geicar-kleya/jetsonNano/ros2_ws/launch.sh`

>[!NOTE]
>More information on the startup and the build are available in the [Project architecture notes](https://docs.google.com/document/d/1LTkkd4-w4RaJ8ROUtFZ62aQJ4dhyVaZO-zE2RoP_Y3U/edit?usp=sharing)

* To turn off the car:
  * Use the red button as a switch to turn off the power.

### Use of this repository
* Have a look in "general" directory for how to connect and work with your car
* For more 'in depth' documentation on a particular subsystem, have a look in following directories:
    * raspberryPI3: Everything about raspberry setup
    * nucleoL476: Stuff about GPS-RTK and IMU
    * nucleoF103: informations on motors control, main power and ultrasonics sensors
    * jetsonNano: Directory containing info on IA, Camera and Lidar
    * simulation: if you want to setup a carla simulation environment

### Building the ROS2 environments

#### On the Raspberry Pi 4
* Clone this repository
* Go into `./raspberryPI3/ros2_ws`
* Run `build.sh`

#### On the Jetson
* Clone this repository
* Go into `./jetsonNano/ros2_ws`
* Run `build.sh`

# Package Summary 

Minimal Example for Using the KUKA LWR, that contains some basic functionalities such as gravity compensation and Joint position Control 


# Dependencies 

- This package uses the Kuka FRI library. This should be properly installed into your workspace, for that, please check the information on the corresponding packages. To do that, clone https://gitlab.lrz.de/research-projects1/frilibrary.git into your workspace 
- We need also the https://gitlab.lrz.de/research-projects1/hierarchical_varimpedance_control.git , to do that, clone and install the package into your following the instructions of the repo 
- We need also the library from the haptic device, clone it also to your workspace 



## Getting Started 

1) To use the package, simply clone the repo. into your catkin_ws/src 


2) Then, update your CMakeLists.txt depending on the lib directories for the H.D and the Kuka, namely the following variables 

        * LIB_FRI: yourpathToFRI/FRILibrary/Linux/x64/release/lib
        * SOURCES_FRI: yourpathToFRI/FRILibrary/src
        * LIB_HD: yourpathToHapticDevSDK/sdk-3.7.3/lib/release/lin-x86_64-gcc
        * INCLUDE_HD: yourpathToHapticDevSDK/sdk-3.7.3/include  
        * INCLUDE_REHEARSE: yourpathToRehearse/include/coin
        * LIB_REHEARSE: yourpathToRehearse/lib

3) You should be ready to build the package with catking_make


## Running The package

1) Make sure that you have sudo rights i.e 
```
sudo su
```
2) Execute the following command:
```
route add -net 192.168.0.20 netmask 255.255.255.255 gw 192.168.0.100 dev enp4s0
```
3) Use the following Command to run the package: 

```
roslaunch kuka_template_new robot_main.launch
```

4) Use the following command to build the package:
<!--  -->

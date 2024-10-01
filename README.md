# Package Summary

- C++ implementation of second-order energy tanks based on control barrier functions, which take into account energy and power constraints, and do not feature the discrete switching behavior typical of energy tanks. For more details, check our recent paper on this:


> [1] Y. Michel, M. Saveriano, and D. Lee, "A Novel Safety-Aware Energy Tank Formulation Based on Control Barrier Functions," IEEE Robotics And Automation Letters (RAL), 2024.

- The package also provides the implementation of several SOA energy tanks algorithms, such as the classical energy tanks (first order), and the first order formulation of energy tanks with control barrier functions. For more details, check https://link.springer.com/chapter/10.1007/978-3-319-20988-3_3 and
https://ieeexplore.ieee.org/document/9669000

- The package also implements the algorithms on real robot hardware (KUKA LWR  4+) for several tasks.

## Dependencies

- For QP optimization, we rely on the `qpOASES` from `coin-or` https://github.com/coin-or/Rehearse
- Follow the instructions provided for installation, and note down the directories the `/include` and `/bin` since we will need them later in our `CMakeLists.txt`

## Getting Started

1) To use the package, simply clone the repo. into your catkin_ws/src


2) Then, update your CMakeLists.txt with the directory of the `qpOASES` library, namely the following variables

        * INCLUDE_QP: yourpathToqpOASES/include
        * LIB_QP: yourpathToqpOASES/bin

3) You should be ready to build the package with catkin_make


## Description
The main scripts in this project are:
-  `src/cbf_tank.cpp` : implementation of the energy tank algorithm proposed in [1]
-  `src/first_order_tank.cpp`: implementation of the first order energy tank (classical) as well as its extension with control barrier functions, which was proposed in https://ieeexplore.ieee.org/document/9669000
- The folder `src/tasks/` contains the ros nodes for real robot implementation. The nodes incoprate the energy tank in the real robot control loop, and therefore are also responsible for communicating with the Kuka low level controller via the FRI library. The implementation should therefore be customized depending on the robot hardware and interface you are using
- Several tasks are implemented to showcase the behavior of the energy tank (check the paper for the details experiments):
    - Variable Stiffness Task: The robot is commanded with a cartesian impedance controller with a smoothly rising variable stiffness profile.
    - Force Control task: Hybrid motion-force control, where the robot applies a downward force in the z direction while performs a linear motion in y
    - Kinetic energy limitation: The tank limits the kinetic energy of the robot beyond a predefined threshold. This is achieved via the power flow limitation feature of the proposed cbf tank
    - The `/config` folder contains the `.yaml` files with the parameters relevant for each task


## Running The package

Use the launch file, where you have to specify
- The task:
   - `force` the force control task
   - `KE` : the kinetic energy limitation task
   - `freemotion`: The variable impedance controller with the smoothly rising stiffness
- The tank type:
   - `cbf`: The proposed CBF tanks in [1]
   - `none`: No energy tank is activated
```
roslaunch cbf_tanks_pkg cbf_tank.launch task:=force tank_type:=cbf
```
- Only for the `freemotion` task, we can further specify other tank types, since the other energy tank approaches are implemnted for this task only (for comparison reasons).
   - `first`: Classical first order tanks
   - `firstcbf` : first order tanks based on cbfs, proposed in https://ieeexplore.ieee.org/document/9669000
```
roslaunch cbf_tanks_pkg cbf_tank.launch task:=force tank_type:=first
```


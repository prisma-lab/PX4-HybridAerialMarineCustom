# PX4_Hybrid_Aerial_Marine_Custom
 Customization of PX4 firmware to introduce a custom flight control stack for autonomous hybrid aerial-amphibious drones.

## Custom PX4 firmware for hybrid aerial-marine autonomous missions
__abstract__ Marine surveys aimed at mapping and monitoring aquatic environments often involve multiple operational phases that can be performed more efficiently, safely, and cost-effectively through the use of autonomous hybrid aerial--amphibious drones. This paper presents an enhanced iteration of the PX4 autopilot firmware specifically tailored for such hybrid platforms, enabling operators to plan and execute missions, including both aerial flight and water-surface navigation, in a fully autonomous manner. The proposed firmware extension introduces two dedicated control modes for manual and autonomous marine navigation while preserving full compatibility with the standard PX4 flight modalities and incorporating the required safety mechanisms. Integration with QGroundControl has also been ensured to support seamless mission planning and mode management. The effectiveness of the proposed functionalities has been validated through simulated case studies.
## Article
The description of the firmware architecture, and the integration with the standard PX4 control stack are described in the following article:

Andrea Capuozzo, Fabio Ruggiero, Vincenzo Lippiello, "Custom PX4 firmware for hybrid aerial-marine autonomous missions", submitted to the 2026 International Conference on Unmanned Aircraft System (ICUAS ’26)  June 15-18, Corfu, Greece

This work is currently under review

## Video


# How to use
Clone the repository with submodules
```sh
git clone --recurse-submodule https://github.com/prisma-lab/PX4-Hybrid_Aerial_Marine_Custom.git
```

## Run the simulation
Set-up simulation environment with the custom Virtual RobotX (VRX) environment
https://github.com/prisma-lab/Virtual_RobotX-VRX.git

To use the PX4 models in the VRX environment run the followings
```sh
export PX4_GZ_MODELS=pathTo/PX4-Autopilot/Tools/simulation/gz/models
export PX4_GZ_WORLDS=pathTo/PX4-Autopilot/Tools/simulation/gz/worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS
```

Start the simulation
```sh
cd pathTo/vrx_ws
. install/setup.bash
ros2 launch vrx_gz WaterTakeOff.launch.py
```

Spawn the Unmanned aerial-acquatic vehicle (UAAV) in the simulation
```sh
cd pathTo/PX4-Autopilot
PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL=USV_test ./build/px4_sitl_default/bin/px4
```
### The custom QGC interface is available here 
https://github.com/prisma-lab/qgroundcontrol.git
<!--
Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.

NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
property and proprietary rights in and to this material, related
documentation and any modifications thereto. Any use, reproduction,
disclosure or distribution of this material and related documentation
without an express license agreement from NVIDIA CORPORATION or
its affiliates is strictly prohibited.
-->
# cuRobo

*CUDA Accelerated Robot Library*

**Check [curobo.org](https://curobo.org) for installing and getting started with examples!**

Use [Discussions](https://github.com/NVlabs/curobo/discussions) for questions on using this package.

Use [Issues](https://github.com/NVlabs/curobo/issues) if you find a bug.


For business inquiries, please visit our website and submit the form: [NVIDIA Research Licensing](https://www.nvidia.com/en-us/research/inquiries/)

## Overview

cuRobo is a CUDA accelerated library containing a suite of robotics algorithms that run significantly faster than existing implementations leveraging parallel compute. cuRobo currently provides the following algorithms: (1) forward and inverse kinematics,
(2) collision checking between robot and world, with the world represented as Cuboids, Meshes, and Depth images, (3) numerical optimization with gradient descent, L-BFGS, and MPPI, (4) geometric planning, (5) trajectory optimization, (6) motion generation that combines inverse kinematics, geometric planning, and trajectory optimization to generate global motions within 30ms.

<p align="center">
<img width="500" src="images/robot_demo.gif">
</p>


cuRobo performs trajectory optimization across many seeds in parallel to find a solution. cuRobo's trajectory optimization penalizes jerk and accelerations, encouraging smoother and shorter trajectories. Below we compare cuRobo's motion generation on the left to a BiRRT planner for the motion planning phases in a pick and place task.

<p align="center">
<img width="500" src="images/rrt_compare.gif">
</p>


## Getting Started
### Installation
1. Clone this repository
```
cd ~/
git clone git@github.com:makolon/curobo.git

```
2. Build docker
```
cd ~/curobo
./docker/build_docker.sh isaac_sim_2023.1.0
```

3. Run docker
```
cd ~/curobo
./docker/start_docker.sh isaac_sim_2023.1.0
```

> [!NOTE]
> In the current version, it is necessary to execute the following command to reflect the changes made to curobot in the source code.
> `cd ~/curobot && omni_python -m pip install .[dev] --no-build-isolation`
> It takes about 10 minutes. 

### Run demo
Firstly, check whether the `omni_python` command can be run.
Then, run the following command.
```
omni_python examples/isaac_sim/motion_gen_reacher.py --robot xarm7.yml --external_robot_configs_path /pkgs/curobo/src/curobo/content/configs/robot/
```

## Citation

If you found this work useful, please cite the below report,

```
@misc{curobo_report23,
      title={cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation}, 
      author={Balakumar Sundaralingam and Siva Kumar Sastry Hari and Adam Fishman and Caelan Garrett 
              and Karl Van Wyk and Valts Blukis and Alexander Millane and Helen Oleynikova and Ankur Handa 
              and Fabio Ramos and Nathan Ratliff and Dieter Fox},
      year={2023},
      eprint={2310.17274},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
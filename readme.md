# A Tutorial for Mujoco For Me

<!--toc:start-->
- [A Tutorial for Mujoco For Me](#a-tutorial-for-mujoco-for-me)
  - [Install](#install)
  - [How to Study](#how-to-study)
  - [The Summary of Mujoco Python tutorial](#the-summary-of-mujoco-python-tutorial)
    - [Creating a Model Instance](#creating-a-model-instance)
      - [mjModel](#mjmodel)
      - [Named access](#named-access)
    - [mjData](#mjdata)
      - [qpos](#qpos)
    - [Rendering](#rendering)
    - [Error handling](#error-handling)
    - [CallBacks](#callbacks)
    - [Simulation](#simulation)
    - [Functions](#functions)
    - [Enums and constants](#enums-and-constants)
    - [Noise](#noise)
  - [A Summary of *dm_control*](#a-summary-of-dmcontrol)
    - [Rendering](#rendering)
  - [Example](#example)
    - [tippe-top](#tippe-top)
    - [chaotic pendulum](#chaotic-pendulum)
    - [Contract](#contract)
    - [Friction](#friction)
    - [Tendons, actuators and sensors](#tendons-actuators-and-sensors)
<!--toc:end-->

## Install

> reference: https://github.com/HILMR/LearnMujoco

Mujoco is an open software now, and it can be installed through **pip**:

```
pip install mujoco
```

> mujoco_py has be deprecated.

It's recommended to install `dm_control` for convenience.

```
pip install dm_control
```

## How to Study

- Official Tutorial for Python:
        - [mujoco](https://github.com/deepmind/mujoco/blob/main/python/tutorial.ipynb)
        - [dm_control](https://github.com/deepmind/dm_control/blob/main/tutorial.ipynb)
- Official Website: [how to write the configuration files and how to use the api](https://mujoco.readthedocs.io/en/latest/overview.html)

## The Summary of Mujoco Python tutorial

> ~~The tutorial is bad... They use mediapy to replay the scene after the simulation and I can not run the demo. Thus, this section will not mention any about how to render the scene in Mujoco interactive viewer.~~
> 
> ops, there is a section introducing the interactive viewer, haha :)

### Creating a Model Instance

1. `mujoco.MjModel.from_xml_string("xml")`: create a binary instance directly from xml description.
> xml string is written in MuJoCo's MjCF, which is an XML-based modeling language.
2. `mujoco.MjModel.from_xml_path(path_to_file)`: create a binary instance from a xml file.
3. `mujoco.MjModel.from_binary_path`:...

A model configuration file example:

```xml
<mujoco>
        <worldbody>
                <light name="top" pos="0 0 1" />
                <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1" />
                <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1" />
        </worldbody>
</mujoco>
```

All physical elements live inside the `<worldbody>`.

> The default position is `0 0 0`, the default geom type type is *sphere*. For more detail, see [here](https://mujoco.readthedocs.io/en/latest/XMLreference.html).

#### mjModel

`MjModel` contains the model description. The complete description can be found at the end of the header file *mjmodel.h*.

example: `model.ngeom`. `model.geom_rgba`

#### Named access

- `model.geom('green_sphere')` will tell us all the valid properties.
- `model.geom('green_sphere').rgba` will give the value of rgba.
- `mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'green_sphere')` will give the id of instance which can be used to indexed like `model.geom_rgba[id, :]`.

> Note that the 0th body is always the world. It cannot be renamed.

### mjData

> These classes provide access to the raw memory used by MuJoCo without copying or buffering. The user is advised to create copies where required, for instance, `position.append(data.body('my_body').xpos.copy())`.
`MjData` contains the state and quantities that depend on it. The state is made up of time, generalized position and generalized velocities. These are respectively `data.time`, `data.qpos` and `data.qvel`. To make a new `MjData`. we can

```py
data = mujoco.MjData(model)
```

To initialize the environment, we need explicitly propagate the derived quantities in `mjData` using `mj_kinematics` which computes global Cartesian poses for all objects (excluding cameras and lights).

```py
mujoco.mj_kinematics(model, data)
```

To propagate the values in `mjData` for dynamic, we need

```py
mujoco.mj_forward(model, data)
```

Next we introduce the common members of mjData.

#### qpos

It is the generalized position. ([pos, quat] for freejoint)

### Rendering

**Standalone application**:

- `python -m mujoco.viewer` launches an empty visualization session.
- `python -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml` launches a visualization session for the model file.

**Manage viewer**:

This function blocks user code to support precise timing of the physics loop.

- `viewer.launch()` launches a empty visualization session.
- `viewer.launch(model)` launches a visualization session for the given `mjModel`.
- `viewer.launch(model, data)` has same function as above except it operates directly on the given `mjData` instance.

**Usage**:

```py
import mujoco.viewer

mujoco.viewer.launch([model, [data]])
```

**Passive viewer**:

This function will not block, allowing user code to continue execution. The user's script is responsible for *timing* and *advancing the physics state*, and *mouse-drag perturbations* will not work.

```py
import time

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('/path/to/mjcf.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
```

`launch_passive` function returns a handle which can be used to interact with the viewer, it has the following attributes:

- `scn`, `cam`, `opt` and `perrt` properties: correspond to *mjvScene, mjvCamera, mjvOption, mjvPerturb* structs respectively.
- `lock()`: User code must ensure that it is holding the viewer lock before modifying any physic or visualization state, including `mhModel` and `mjData`, and above four properties of viewer.
- `sync()`: synchronizes state between `mjModel`, `mjData` and GUI user inputs since the previous call to `sync`.
- `close()`: closes the viewer window.
- `is_runing()`: return `True` is the viewer window is runnig.

> Flow: First, get a policy and then make a step, [change any state after lock] and then run `sync` and wait until the rest time running out.

In order to calling any of `mjr_` rendering routine, users are expected to set up a working OpenGL context, for instance,

```py
ctx = mujoco.GLContext(max_width, max_height)
ctx.make_current()

.
.
.

ctx.free() # free the context.
```

### Error handling

...

### CallBacks

MuJoCo allows users to install custom callback functions to modify certain parts of its computation pipeline. For example, `mjcb_sensor` can be used to implement custom sensors, and `mjcb_control` can be used to implement custom actuators.

### Simulation

Using MuJoCo's main high level function `mj_step` to step the state $x_{t+h} = f(x_t)$.

```py
mujoco.mj_step(model, data)
```

To make things move, we need to add DoFs to model by adding *joints* to bodies, specifying how they can move with respect to their parents.

Example:

```xml
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1"/>
    <option gravity="0 0 -9.81" />
    <body name="box_and_sphere" euler="0 0 -30">
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
```

To visualize the joint axis, one can use visualization option object `MjOption`.

```py
scene_option = mujoco.MjvOption()
scene_option.flagsp[mujoco.mjtVisFlag.mjVIS_JOINT] = True
```

Physics options live in mhModel.opt, for example: `model.opt.timestep`, `model.opt.gravity`.

> The options can be modified. We can modify these in XML using the top-level `<option>` elements

MuJoCo uses a representation known as the "Lagrangian", "generalized" or "additive" representation, whereby objects have no DoFs unless explicitly added using joints.

```py
print('Total number of DoFs in the model:', model.nv)
print('Generalized positions:', data.qpos)
print('Generalized velocities:', data.qvel)
```

### Functions

MuJoCo functions are exposed as Python functions of the same name. In Python, the size arguments are omitted since we can automatically (and indeed, more safely) deduce it from the NumPy array. When calling these functions, pass all arguments other than array sizes in the same order as they appear in [mujoco.h](https://github.com/deepmind/mujoco/blob/main/include/mujoco/mujoco.h).

### Enums and constants

MuJoCo enums are available as `mujoco.mjtEnumType.ENUM_VALUE`, for example `mujoco.mjtObj.mjOBJ_SITE`. MuJoCo constants are available with the same name directly under the mujoco module, for example `mujoco.mjVISSTRING`.

### Noise

MuJoCo simulations are deterministic with one exception: sensor noise can be generated when this feature is enabled. This is done by calling the C function rand() internally. To generate the same random number sequence, call srand() with a desired seed after the model is loaded and before the simulation starts. The model compiler calls srand(123) internally, so as to generate random dots for procedural textures. Therefore the noise sequence in the sensor data will change if the specification of procedural textures changes, and the user does not call srand() after model compilation.

## A Summary of *dm_control*

### Rendering

```py
from dm_control import suite
from dm_control import viewer

# Load an environment from the Control Suite.
env = suite.load(domain_name="humanoid", task_name="stand")

# Launch the viewer application.
viewer.launch(env)
```

```py
from dm_control import suite
from dm_control import viewer
import numpy as np

env = suite.load(domain_name="humanoid", task_name="stand")
action_spec = env.action_spec()

# Define a uniform random policy.
def random_policy(time_step):
  del time_step  # Unused.
  return np.random.uniform(low=action_spec.minimum,
                           high=action_spec.maximum,
                           size=action_spec.shape)

# Launch the viewer application.
viewer.launch(env, policy=random_policy)
```

## Example

### tippe-top

```xml
<mujoco model="tippe top">
  <option integrator="RK4"/>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".2"/>
  </asset>

  <worldbody>
    <geom size=".2 .2 .01" type="plane" material="grid"/>
    <light pos="0 0 .6"/>
    <camera name="closeup" pos="0 -.1 .07" xyaxes="1 0 0 0 1 2"/>
    <body name="top" pos="0 0 .02">
      <freejoint/>
      <geom name="ball" type="sphere" size=".02" />
      <geom name="stem" type="cylinder" pos="0 0 .02" size="0.004 .008"/>
      <geom name="ballast" type="box" size=".023 .023 0.005"  pos="0 0 -.015"
       contype="0" conaffinity="0" group="3"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="spinning" qpos="0 0 0.02 1 0 0 0" qvel="0 0 0 0 1 200" />
  </keyframe>
</mujoco>
```
> `qpos` has 7 elements, first three are position of body, the rest are unit quaternion.

Notes:

1. Using `<option/>` clause to set the integrator.
2. Using `<asset/>` clause to define floor's grid material.
3. Using `<freejoint/>` clause to add 6-Dof joint.
4. We use an invisible and non-colliding box geom called `ballast` to move the top's center-of-mass lower. Having a low center of mass is (counter-intuitively) required for the flipping behavior to occur.
5. We save our initial spinning state as a *keyframe*. It has a high rotational velocity around the Z-axis, but is not perfectly oriented with the world, which introduces the symmetry-breaking required for the flipping.
> keyframe can be set by `mujoco.mj_resetDataKeyframe(model, data, 0)`.
6. We define a `<camera>` in our model, and then render from it using the `camera` argument to `update_scene()`.

### chaotic pendulum

...

### Contract

...

### Friction

```xml
<mujoco>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="none"/>
    <material name="grid" texture="grid" texrepeat="6 6"
     texuniform="true" reflectance=".2"/>
     <material name="wall" rgba='.5 .5 .5 1'/>
  </asset>

  <default>
    <geom type="box" size=".05 .05 .05" />
    <joint type="free"/>
  </default>

  <worldbody>
    <light name="light" pos="-.2 0 1"/>
    <geom name="ground" type="plane" size=".5 .5 10" material="grid"
     zaxis="-.3 0 1" friction=".1"/> 
    <camera name="y" pos="-.1 -.6 .3" xyaxes="1 0 0 0 1 2"/>
    <body pos="0 0 .1">
      <joint/>
      <geom/>
    </body>
    <body pos="0 .2 .1">
      <joint/>
      <geom friction=".33"/>
    </body>
  </worldbody>

</mujoco>
```

### Tendons, actuators and sensors

```xml
<mujoco>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3"
     rgb2=".2 .3 .4" width="300" height="300" mark="none"/>
    <material name="grid" texture="grid" texrepeat="1 1"
     texuniform="true" reflectance=".2"/>
  </asset>

  <worldbody>
    <light name="light" pos="0 0 1"/>
    <geom name="floor" type="plane" pos="0 0 -.5" size="2 2 .1" material="grid"/>
    <site name="anchor" pos="0 0 .3" size=".01"/>
    <camera name="fixed" pos="0 -1.3 .5" xyaxes="1 0 0 0 1 2"/>

    <geom name="pole" type="cylinder" fromto=".3 0 -.5 .3 0 -.1" size=".04"/>
    <body name="bat" pos=".3 0 -.1">
      <joint name="swing" type="hinge" damping="1" axis="0 0 1"/>
      <geom name="bat" type="capsule" fromto="0 0 .04 0 -.3 .04"
       size=".04" rgba="0 0 1 1"/>
    </body>

    <body name="box_and_sphere" pos="0 0 0">
      <joint name="free" type="free"/>
      <geom name="red_box" type="box" size=".1 .1 .1" rgba="1 0 0 1"/>
      <geom name="green_sphere"  size=".06" pos=".1 .1 .1" rgba="0 1 0 1"/>
      <site name="hook" pos="-.1 -.1 -.1" size=".01"/>
      <site name="IMU"/>
    </body>
  </worldbody>

  <tendon>
    <spatial name="wire" limited="true" range="0 0.35" width="0.003">
      <site site="anchor"/>
      <site site="hook"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="my_motor" joint="swing" gear="1"/>
  </actuator>

  <sensor>
    <accelerometer name="accelerometer" site="IMU"/>
  </sensor>
</mujoco>
```

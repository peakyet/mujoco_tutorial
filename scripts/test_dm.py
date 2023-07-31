#@title All `dm_control` imports required for this tutorial

# The basic mujoco wrapper.
from dm_control import mujoco

# Access to enums and MuJoCo library functions.
from dm_control.mujoco.wrapper.mjbindings import enums
from dm_control.mujoco.wrapper.mjbindings import mjlib

# PyMJCF
from dm_control import mjcf

# Composer high level imports
from dm_control import composer
from dm_control.composer.observation import observable
from dm_control.composer import variation

# Imports for Composer tutorial example
from dm_control.composer.variation import distributions
from dm_control.composer.variation import noises
from dm_control.locomotion.arenas import floors

# Control Suite
from dm_control import suite

# Run through corridor example
from dm_control.locomotion.walkers import cmu_humanoid
from dm_control.locomotion.arenas import corridors as corridor_arenas
from dm_control.locomotion.tasks import corridors as corridor_tasks

# Soccer
from dm_control.locomotion import soccer

# Manipulation
from dm_control import manipulation


# Load a model from an MJCF XML string.
xml_string = """
<mujoco>
  <worldbody>
    <light name="top" pos="0 0 1.5"/>
    <geom name="floor" type="plane" size="1 1 .1"/>
    <body name="box" pos="0 0 .3">
      <joint name="up_down" type="slide" axis="0 0 1"/>
      <geom name="box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <geom name="sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""
physics = mujoco.Physics.from_xml_string(xml_string)

# Render the default camera view as a numpy array of pixels.
pixels = physics.render()

# Reset the simulation, move the slide joint upwards and recompute derived
# quantities (e.g. the positions of the body and geoms).
with physics.reset_context():
  physics.named.data.qpos['up_down'] = 0.5

# Print the positions of the geoms.
print(physics.named.data.geom_xpos)
# FieldIndexer(geom_xpos):
#            x         y         z
# 0  floor [ 0         0         0       ]
# 1    box [ 0         0         0.8     ]
# 2 sphere [ 0.2       0.2       1       ]

# Advance the simulation for 1 second.
while physics.time() < 1.:
  physics.step()

# Print the new z-positions of the 'box' and 'sphere' geoms.
print(physics.named.data.geom_xpos[['box', 'sphere'], 'z'])
# [ 0.19996362  0.39996362]

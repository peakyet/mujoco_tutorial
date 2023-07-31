import numpy as np
import time
import mujoco
import mujoco.viewer

path = "./Models/ball.xml"

def main():
    model = mujoco.MjModel.from_xml_path(path)

    data = mujoco.MjData(model)

    # Cartesian positions of objects
    print(f"data.geom_xpos: {data.geom_xpos}")    

    # mujoco.viewer.launch(model, data)

    Passive_viewer(model, data)

def Passive_viewer(model,data):
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close the viewer automatically

        mujoco.mj_step(model, data)
        start = time.time()
        while viewer.is_running():
            step_start = time.time()

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            qpos = data.qpos
            print(f"qpos: {qpos}")
            stateOfBall = qpos[7:]
            goal = qpos[:3]
            force = ball_tracker(stateOfBall, goal)
            data.ctrl[4] = force[0]
            data.ctrl[5] = force[1]
            data.ctrl[6] = force[2]
            mujoco.mj_step(model, data)

            # print(f"data.geom_xpos: {data.geom_xpos}")
            # print(f"data.qpos: {data.qpos}")

            # Example modification of a viewer option: toggle contact points every two seconds.
            # with viewer.lock():
            #     viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step=model.opt.timestep - (time.time() - step_start)

            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

def ball_tracker(state, goal):
    # uMax = 1.0
    force = (goal - state)
    return force

if __name__ == "__main__":
    main()

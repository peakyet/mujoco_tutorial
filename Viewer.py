import time
import mujoco
import mujoco.viewer

path = "./double_pendulum.xml"

def main():
    model = mujoco.MjModel.from_xml_path(path)

    # The description of object
    shoulder = model.geom("shoulder")
    print(f"shoulder: {shoulder}")

    data = mujoco.MjData(model)

    # Cartesian positions of objects
    print(f"data.geom_xpos: {data.geom_xpos}")    

    # mujoco.viewer.launch(model, data)

    Passive_viewer(model, data)

def Passive_viewer(model,data):
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close the viewer automatically

        start = time.time()
        while viewer.is_running():
            step_start = time.time()

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(model, data)

            print(f"data.geom_xpos: {data.geom_xpos}")    

            # Example modification of a viewer option: toggle contact points every two seconds.
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step=model.opt.timestep - (time.time() - step_start)

            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
    

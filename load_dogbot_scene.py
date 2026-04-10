import omni
import time
import numpy as np
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
import omni.kit.commands

class Go2Sim():
    def __init__(self):   
        # initial settings
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(physics_dt = 0.001, rendering_dt = 0.025, stage_units_in_meters=1)
        self.world.add_physics_callback("callback_control", callback_fn=self.callback)
        
        # add surface and robot dog
        self.world.scene.add_default_ground_plane()
        self.robot_asset_path = "D:\\10102127\\go2_description\\urdf\\go2_description\\go2_description.usd"
        self.prim_path = "/World/Go2"
        add_reference_to_stage(usd_path=self.robot_asset_path, prim_path=self.prim_path)

        self.Go2 = Robot(prim_path="/World/Go2", name="Go2", position=np.array([0.0, 0.0, 0.5]), orientation=np.array([0, 0, 0, 1]))
        self.world.scene.add(self.Go2)
        
        #reset the world to default, ***SENSOR HAVE TO DECLARE BEFORE IT!!!***
        self.world.reset()

        self.sim_loop_freq = 40

    def callback(self, step_size):
        print(1)

    def run_simulation(self):
        self.world.reset()
        self.timeline.play()
    
        self.Go2.set_joint_positions(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    
        while simulation_app.is_running():
            start = time.time()

            #update simulation step
            self.world.step(render=True)
            
            #check if delay is needed or not
            end = time.time()
            if (end - start < 1/self.sim_loop_freq):
                time.sleep(1/self.sim_loop_freq - (end - start))
            print("Finish 1 render")

        # Cleanup
        self.timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    go2_sim = Go2Sim()
    go2_sim.run_simulation()

import omni
import time
import numpy as np
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "sync_loads": True,
    "active_config": "default"
})

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot

class Go2Sim():
    def __init__(self):   
        # initial settings
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.set_auto_update(True)
        self.world = World(physics_dt = 0.002, rendering_dt = 0.025, stage_units_in_meters=1)
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

    def callback(self, step_size):
        print(1)

    def run_simulation(self):
        self.world.reset()
        self.timeline.play()
    
        self.Go2.set_joint_positions(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        start = 0
        end = 0
        while simulation_app.is_running():
            end = start
            start = time.time()
            print(start - end)
            #update simulation step
            self.world.step(render=True)
            
            #check if delay is needed or not            

        # Cleanup
        self.timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    go2_sim = Go2Sim()
    go2_sim.run_simulation()

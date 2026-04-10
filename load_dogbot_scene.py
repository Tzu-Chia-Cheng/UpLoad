import omni
import time
import numpy as np
from isaacsim import SimulationApp

#It is mandated that it be written this way.
simulation_app = SimulationApp( { "headless": False } )

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot

class Go2Sim():
    def __init__(self):   
        # initial settings
        self.Timeline = omni.timeline.get_timeline_interface()
        
        self.World = World( physics_dt = 0.002, rendering_dt = 0.02, stage_units_in_meters = 1 )
        
        # add surface and robot dog
        self.World.scene.add_default_ground_plane()
        self.RobotAssetPath = "D:\\10102127\\go2_description\\urdf\\go2_description\\go2_description.usd"
        self.PrimPath = "/World/Go2"
        add_reference_to_stage( usd_path = self.RobotAssetPath, prim_path = self.PrimPath )

        self.Go2 = Robot(prim_path = self.PrimPath, name = "Go2", position = np.array( [ 0.0, 0.0, 0.5 ] ) )
        self.World.scene.add(self.Go2)
        
        #reset the world to default, ***SENSOR HAVE TO DECLARE BEFORE IT!!!***
        self.World.reset()

    def run_simulation(self):
        self.World.reset()
        self.Timeline.play()
        InitPos = np.zeros( 12 )
        self.Go2.set_joint_positions( InitPos )

        while simulation_app.is_running():
            #update simulation step
            self.World.step(render=True)         

        # Cleanup
        self.Timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    go2_sim = Go2Sim()
    go2_sim.run_simulation()

import numpy as np
import omni
import time
import math
import csv
from pathlib import Path
from scipy.spatial.transform import Rotation as R

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

simulation_app.update()

from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from isaacsim.sensors.physics import _sensor
import omni.kit.commands
from pxr import Gf

class DogBotSim():
    def __init__(self):

        # initial settings
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(physics_dt = 0.001, rendering_dt = 0.001, stage_units_in_meters=1)

        # add surface and robot dog
        self.world.scene.add_default_ground_plane()
        self.robot_asset_path = "D:\\10102127\\go2_description\\urdf\\go2_description\\go2_description.usd"
        self.prim_path = "/World/Dogbot"
        add_reference_to_stage(usd_path=self.robot_asset_path, prim_path=self.prim_path)

        self.dogbot = Robot(prim_path="/World/Dogbot", name="Dogbot", position=np.array([2.4, 0.0, 0.57]), orientation=np.array([0, 0, 0, 1]))
        self.world.scene.add(self.dogbot)
        
        #reset the world to default, ***SENSOR HAVE TO DECLARE BEFORE IT!!!***
        self.world.reset()

        # define robot dog as robot
        self.dogbot_articulation_controller = self.dogbot.get_articulation_controller()
        self.num_dof = self.dogbot.num_dof
        self.dof_names = self.dogbot.dof_names
        self._joint_cmd = [0.0] * (self.num_dof + 1)
        self.target_joint_cmd = np.zeros(self.num_dof)
        
        #setup control articulation
        self.target_action = ArticulationAction()

        self.sim_loop_freq = 100

    def run_simulation(self):
        self.world.reset()
        self.timeline.play()
    
        self.dogbot.set_joint_positions(np.array([0.0, 0.0, 0.0, 0.0, 0.14130318048264423, -0.14130318048264423, 0.8253112497038592, -0.8253112497038592, -0.6625383660716182, 0.6625383660716182, -0.14856298508118196, 0.14856298508118196]))
    
        while simulation_app.is_running():
            start = time.time()

            #update simulation step
            self.world.step(render=True)
            
            #check if delay is needed or not
            end = time.time()
            if (end - start < 1/self.sim_loop_freq):
                time.sleep(1/self.sim_loop_freq - (end - start))

        # Cleanup
        self.timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    dogbot_sim = DogBotSim()
    dogbot_sim.run_simulation()

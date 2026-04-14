import omni
import time
import numpy as np
from isaacsim import SimulationApp

#It is mandated that it be written this way.
simulation_app = SimulationApp( { "headless": False } )

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.physics import _sensor
import omni.kit.commands
from pxr import Gf

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_isaac_sim_bridge import CGo2IsaacSimBridge
import threading

from IsaacDataDef import CIsaacData
import isaacsim_config as config

IsaacData = CIsaacData()

class Go2Sim():
    def __init__(self):   
        # isaac sim initial settings
        self.Timeline = omni.timeline.get_timeline_interface()

        self.World = World( physics_dt = config.SIM_PHYSICS_DT, rendering_dt = config.SIM_RENDER_DT, stage_units_in_meters = 1 )
        self.World.add_physics_callback( "callback_control", callback_fn = self.PhysicsCallback )

        # add surface and robot dog
        self.World.scene.add_default_ground_plane()
        add_reference_to_stage( usd_path = config.ROBOT_SCENE, prim_path = config.ROBOT_PRIM_PATH )

        self.Go2 = Robot(prim_path = config.ROBOT_PRIM_PATH, name = "Go2", position = np.array( [ 0.0, 0.0, 0.0 ] ) )
        self.World.scene.add(self.Go2)
        
        # add IMU sensor
        self.AddIMUSensor()

        # add Foot Sensor
        self.AddFootSensor()

        # reset the world to default, ***SENSOR HAVE TO DECLARE BEFORE IT!!!***
        self.World.reset()

        # add controller and controller action
        self.AddArticulationController()
        
        # set up init pos
        self.InitPos = np.array( [0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 0.8, 0.8, -1.5, -1.5, -1.5, -1.5] )
        
        # set up global variable
        global IsaacData
        
        # initialize dds channel
        self.locker = threading.Lock()
        ChannelFactoryInitialize( config.DOMAIN_ID, config.INTERFACE )
        self.Go2IsaacSimBridge = CGo2IsaacSimBridge( config.SIM_PHYSICS_DT, IsaacData )
    
    def AddIMUSensor( self ):
        self.IMUSensorInterface = _sensor.acquire_imu_sensor_interface()
        self.IMUSetResult, self.IMUSensorPrim = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path = "/IMUSensor",
            parent = "/World/Go2/base",
            sensor_period = -1.0,
            translation = Gf.Vec3d( 0, 0, 0 ),
            orientation = Gf.Quatd( 1, 0, 0, 0 ),
        )
    
    def AddFootSensor( self ):
        self.FLContactSensorInterface = _sensor.acquire_contact_sensor_interface()
        self.FLContactSensorSetResult, self.FLContactSensorPrim = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="ContactSensor",
            parent="/World/Go2/FL_foot",
            sensor_period = -1,
            min_threshold = 0.0,
            max_threshold = 500,
            translation = Gf.Vec3d(0, 0, 0),
        )
        
        self.FRContactSensorInterface = _sensor.acquire_contact_sensor_interface()
        self.FRContactSensorSetResult, self.FRContactSensorPrim = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="ContactSensor",
            parent="/World/Go2/FR_foot",
            sensor_period = -1,
            min_threshold = 0.0,
            max_threshold = 500,
            translation = Gf.Vec3d(0, 0, 0),
        )
        
        self.RLContactSensorInterface = _sensor.acquire_contact_sensor_interface()
        self.RLContactSensorSetResult, self.RLContactSensorPrim = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="ContactSensor",
            parent="/World/Go2/RL_foot",
            sensor_period = -1,
            min_threshold = 0.0,
            max_threshold = 500,
            translation = Gf.Vec3d(0, 0, 0),
        )
        
        self.RRContactSensorInterface = _sensor.acquire_contact_sensor_interface()
        self.RRContactSensorSetResult, self.RRContactSensorPrim = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="ContactSensor",
            parent="/World/Go2/RR_foot",
            sensor_period = -1,
            min_threshold = 0.0,
            max_threshold = 500,
            translation = Gf.Vec3d(0, 0, 0),
        )
    
    def AddArticulationController( self ):
        self.ArticulationController = self.Go2.get_articulation_controller()
        self.TargetAction = ArticulationAction()

        # set up the motor gain Kp, Kd and target motor command
        self.ArticulationController.set_gains(400, 10)
        self.MotorCmd = np.zeros(12)
    
    def PhysicsCallback( self, step_size ):
        self.locker.acquire()
        
        self.GetSensorData()
        self.MotorCmd = IsaacData.Cmd
        
        self.TargetAction.joint_positions  = None
        self.TargetAction.joint_efforts    = self.MotorCmd
        self.TargetAction.joint_velocities = None
        self.TargetAction.joint_indices    = None

        self.ArticulationController.apply_action(self.TargetAction)

        self.locker.release()
        
    def GetSensorData( self ):
        RecvMotorAng = self.Go2.get_joint_positions()
        RecvMotorVel = self.Go2.get_joint_velocities()
        RecvMotorEffort = self.Go2.get_measured_joint_efforts()
        IMUData = self.IMUSensorInterface.get_sensor_reading( "/World/Go2/base/IMUSensor" )
        FLContactSensorData = self.FLContactSensorInterface.get_sensor_reading( "/World/Go2/FL_foot/ContactSensor", use_latest_data = True )
        FRContactSensorData = self.FRContactSensorInterface.get_sensor_reading( "/World/Go2/FR_foot/ContactSensor", use_latest_data = True )
        RLContactSensorData = self.RLContactSensorInterface.get_sensor_reading( "/World/Go2/RL_foot/ContactSensor", use_latest_data = True )
        RRContactSensorData = self.RRContactSensorInterface.get_sensor_reading( "/World/Go2/RR_foot/ContactSensor", use_latest_data = True )
        
        IsaacData.SensorData[:config.MOTOR_NUM_GO2] = RecvMotorAng
        IsaacData.SensorData[ config.MOTOR_NUM_GO2 : config.MOTOR_NUM_GO2 * 2 ] = RecvMotorVel
        IsaacData.SensorData[ config.MOTOR_NUM_GO2 * 2 : config.MOTOR_NUM_GO2 * 3 ] = RecvMotorEffort
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 0 ] = IMUData.orientation[0]
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 1 ] = IMUData.orientation[1]
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 2 ] = IMUData.orientation[2]
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 3 ] = IMUData.orientation[3]
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 4 ] = IMUData.ang_vel_x
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 5 ] = IMUData.ang_vel_y
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 6 ] = IMUData.ang_vel_z
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 7 ] = IMUData.lin_acc_x
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 8 ] = IMUData.lin_acc_y
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 9 ] = IMUData.lin_acc_z
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 0 ] = FLContactSensorData.value
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 1 ] = FRContactSensorData.value
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 2 ] = RLContactSensorData.value
        IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 3 ] = RRContactSensorData.value

    def run_simulation(self):
        self.World.reset()
        self.Timeline.play()
        self.Go2.set_joint_positions( self.InitPos )

        while simulation_app.is_running():
            #update simulation step
            StepStart = time.perf_counter()

            self.World.step(render=True)
            
            StepEnd = time.perf_counter()
            TimeUntilNextStep = config.SIM_TIME_STEP - ( StepEnd - StepStart )

            if( TimeUntilNextStep > 0 ):
                time.sleep( TimeUntilNextStep )

        # Cleanup
        self.Timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    go2_sim = Go2Sim()
    go2_sim.run_simulation()
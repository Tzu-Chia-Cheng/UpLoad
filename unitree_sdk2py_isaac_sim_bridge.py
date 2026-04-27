import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

import isaacsim_config as config

class CGo2IsaacSimBridge:
    def __init__( self, PhysicsDt, IsaacData ):
        self.IsaacData = IsaacData
        
        self.MotorCmdFull = []
        for nMotorCnt in range( config.MOTOR_NUM_GO2 ):
            self.MotorCmdFull.append( { "q": 0.0, "dq": 0.0, "kp": 0.0, "kd": 0.0, "tau": 0.0 } )

        # unitree sdk2 message
        self.LowState = LowState_default()
        self.LowStatePuber = ChannelPublisher( config.TOPIC_LOWSTATE, LowState_ )
        self.LowStatePuber.Init()
        self.LowStateThread = RecurrentThread( interval = PhysicsDt, target = self.PublishLowState, name="sim_lowstate" )
        self.LowStateThread.Start()
        
        self.LowCmdSuber = ChannelSubscriber( config.TOPIC_LOWCMD, LowCmd_ )
        self.LowCmdSuber.Init(self.LowCmdHandler, 10)
        
    def LowCmdHandler( self, msg: LowCmd_ ):
        if self.IsaacData != None:
            for nMotorIndex in range( config.MOTOR_NUM_GO2 ):
                # default use torque control
                cmd = msg.motor_cmd[ nMotorIndex ]
                self.MotorCmdFull[ nMotorIndex ][ "q" ]   = cmd.q
                self.MotorCmdFull[ nMotorIndex ][ "dq" ]  = cmd.dq
                self.MotorCmdFull[ nMotorIndex ][ "kp" ]  = cmd.kp
                self.MotorCmdFull[ nMotorIndex ][ "kd" ]  = cmd.kd
                self.MotorCmdFull[ nMotorIndex ][ "tau" ] = cmd.tau
            
            # PD control: tau = kp*(q_des - q_cur) + kd*(dq_des - dq_cur) + tau_ff
            for nMotorIndex in range( config.MOTOR_NUM_GO2 ):
                m = self.MotorCmdFull[ nMotorIndex ]
                q_cur  = self.IsaacData.SensorData[ nMotorIndex ]
                dq_cur = self.IsaacData.SensorData[ nMotorIndex + config.MOTOR_NUM_GO2 ]
                tau = m["kp"] * ( m["q"] - q_cur ) + m["kd"] * ( m["dq"] - dq_cur ) + m["tau"]
                self.IsaacData.Cmd[ nMotorIndex ] = tau
                
    def PublishLowState( self ):
        if self.IsaacData != None:
            # store motor state to publish msg
            for nMotorIndex in range( config.MOTOR_NUM_GO2 ):
                self.LowState.motor_state[ nMotorIndex ].q = self.IsaacData.SensorData[ nMotorIndex ]
                self.LowState.motor_state[ nMotorIndex ].dq = self.IsaacData.SensorData[ nMotorIndex + config.MOTOR_NUM_GO2 ]
                self.LowState.motor_state[ nMotorIndex ].tau_est = self.IsaacData.SensorData[ nMotorIndex + 2 * config.MOTOR_NUM_GO2 ]
            
            # store imu state to publish msg
            self.LowState.imu_state.quaternion[ 0 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 0 ]
            self.LowState.imu_state.quaternion[ 1 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 1 ]
            self.LowState.imu_state.quaternion[ 2 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 2 ]
            self.LowState.imu_state.quaternion[ 3 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 3 ]

            self.LowState.imu_state.gyroscope[ 0 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 4 ]
            self.LowState.imu_state.gyroscope[ 1 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 5 ]
            self.LowState.imu_state.gyroscope[ 2 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 6 ]

            self.LowState.imu_state.accelerometer[ 0 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 7 ]
            self.LowState.imu_state.accelerometer[ 1 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 8 ]
            self.LowState.imu_state.accelerometer[ 2 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + 9 ]
            
            # store foot sensor state to publish msg
            self.LowState.foot_force[ 0 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 0 ]
            self.LowState.foot_force[ 1 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 1 ]
            self.LowState.foot_force[ 2 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 2 ]
            self.LowState.foot_force[ 3 ] = self.IsaacData.SensorData[ config.MOTOR_SENSOR_DIM + config.IMU_SENSOR_DIM + 3 ]
            
            # pub state to hardware interface
            self.LowStatePuber.Write( self.LowState )


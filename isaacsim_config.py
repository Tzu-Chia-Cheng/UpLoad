ROBOT_SCENE = "D:\\10102127\\go2_description\\urdf\\go2_description\\go2_description.usd"
ROBOT_PRIM_PATH = "/World/Go2"

DOMAIN_ID = 1 # domain id
INTERFACE = "lo" # interface

SIM_TIME_STEP = 0.02 # [sec]
SIM_PHYSICS_DT = 0.001 # [sec]
SIM_RENDER_DT = 0.02 # [sec]

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"

MOTOR_SENSOR_NUM = 3
MOTOR_NUM_GO2 = 12
IMU_SENSOR_DIM = 10
MOTOR_SENSOR_DIM = MOTOR_SENSOR_NUM * MOTOR_NUM_GO2

PRINT_SENSOR_DATA = False
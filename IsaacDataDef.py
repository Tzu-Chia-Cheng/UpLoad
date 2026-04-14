import numpy as np
from dataclasses import dataclass, field

@dataclass
class CCmd:
    # 12 motor cmd 
    data: np.ndarray = field( default_factory = lambda: np.zeros( 12 ) )
    
    def __getitem__( self, index ):
        return self.data[index]
    
    def __setitem__(self, index, value):
        self.data[index] = value

@dataclass
class CSensorData:
    # 12 motor position, 12 motor velocity, 12 motor torque, 10 imu data, 4 foot sensor = 50 data
    data: np.ndarray = field( default_factory = lambda: np.zeros( 50 ) )
    
    def __getitem__( self, index ):
        return self.data[index]
    
    def __setitem__(self, index, value):
        self.data[index] = value

@dataclass
class CIsaacData:
    # isaac sim full data
    Cmd: CCmd = field( default_factory = CCmd )
    SensorData: CSensorData = field( default_factory = CSensorData )
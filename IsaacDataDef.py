from dataclasses import dataclass, field
from typing import List

@dataclass
class CCmd:
    # 12 motor cmd 
    data: List[float] = field( default_factory = lambda: [0.0] * 12 )
    
    def __getitem__( self, index ):
        if not 0 <= index < 12:
            raise IndexError(f"Cmd index out of range; expected 0-11, but got {index}.")
        return self.data[index]
    
    def __setitem__(self, index, value):
        if not 0 <= index < 12:
            raise IndexError(f"Cmd index out of range; expected 0-11, but got {index}.")
        self.data[index] = value

@dataclass
class CSensorData:
    # 12 motor position, 12 motor velocity, 12 motor torque, 10 imu data, 4 foot sensor = 50 data
    data: List[float] = field( default_factory=lambda: [0.0] * 50 )
    
    def __getitem__( self, index ):
        if not 0 <= index < 50:
            raise IndexError(f"Cmd index out of range; expected 0-49, but got {index}.")
        return self.data[index]
    
    def __setitem__(self, index, value):
        if not 0 <= index < 50:
            raise IndexError(f"Cmd index out of range; expected 0-49, but got {index}.")
        self.data[index] = value

@dataclass
class CIsaacData:
    # isaac sim full data
    Cmd: CCmd = field( default_factory = CCmd )
    SensorData: CSensorData = field( default_factory = CSensorData )
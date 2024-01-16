from robotiq_modbus_controller.request import Request as RobotiqModbusRtuRequest
from robotiq_modbus_controller.status import Status as RobotiqModbusRtuStatus
from pathlib import Path
import subprocess
import minimalmodbus as mm
from dataclasses import dataclass
import struct

class LinuxFindTTYWithSerialNumber:
    def __init__(self):
        pass

    def find(self, serial_number):
        '''
        Iterate over all /dev/ttyUSB* devices and try to find the one with the given serial number.

        A list of devices can be obtained with: ls /dev/ttyUSB*
        The serial number of a given device can be obtained with: udevadm info -a -n /dev/ttyUSB0 | grep 'ATTRS{serial}' | head -n1
        producing the output: ATTRS{serial}=="serial_number".

        Parameters:
        -----------
        serial_number : str
            Serial number of the device to find.

        Returns:
        --------
        str
            The path to the device with the given serial number or None if no device matching the specified serial number was found.
        '''

        # Get the list of all /dev/ttyUSB* devices.
        tty_devices = Path('/dev').glob('ttyUSB*')

        # Iterate over all /dev/ttyUSB* devices and try to find the one with the given serial number.
        for tty_device in tty_devices:
            # Get the serial number of the current device.
            current_serial_number = self.get_serial_number(tty_device)

            # Check if the serial number of the current device matches the specified serial number.
            if current_serial_number == serial_number:
                # Return the path to the current device.
                return str(tty_device)

        # Return None if no device matching the specified serial number was found.
        return None
    
    def get_serial_number(self, tty_device:Path):
        '''
        Get the serial number of a given device.

        Parameters:
        -----------
        tty_device : Path
            Path to the device.

        Returns:
        --------
        str
            The serial number of the device.
        '''

        # Get the serial number of the device by running the following command:
        # udevadm info -a -n /dev/ttyUSB0 | grep 'ATTRS{serial}' | head -n1
        # producing the output: ATTRS{serial}=="serial_number".
        output = subprocess.check_output(['udevadm', 'info', '-a', '-n', str(tty_device)]).decode('utf-8')
        output = output.split('\n')
        output = [line for line in output if 'ATTRS{serial}' in line]

        #If no line with 'ATTRS{serial}' was found, return None.
        if len(output) == 0:
            return None
        
        try:
            serial_number = output[0].split('"')[1]
        except:
            serial_number = None

        # Return the serial number of the device.
        return serial_number
    
@dataclass
class GripperFault:
    #Reactivation must be performed before any further movement.
    reactivation_required: bool
    #Activation bit must be set prior to action.
    activation_required: bool
    #Gripper's temperature has risen too high and it needs to cool down.
    overheating: bool
    #The voltage supplied to the gripper is below 21.6 Volts
    undervoltage: bool
    #Automatic release in progress
    is_auto_releasing: bool
    #Automatic release completed
    auto_release_completed: bool
    #Internal fault, contact manufacturer.
    internal_fault: bool
    #Activation fault
    activation_fault: bool
    #A current of more than 1 Amp. was supplied
    overcurrent: bool

@dataclass
class GripperStatus:
    activated: bool
    moving: bool
    current: float
    obj_detected: bool
    position: float
    goal_position: float
    is_reset: bool
    is_activating: bool
    is_activated: bool
    fault: GripperFault = GripperFault(0,0,0,0,0,0,0,0,0)


class Robotiq2F85Driver:
    def __init__(self, serial_number:str):
        self.device_serial_number = serial_number
        self.tty_device = LinuxFindTTYWithSerialNumber().find(serial_number)
        
        if self.tty_device is None:
            raise Exception('No device with serial number {} found.'.format(serial_number))
        
        self.client = mm.Instrument(port=self.tty_device, slaveaddress=9, mode=mm.MODE_RTU)
        self.client.serial.baudrate = 115200
        self.client.serial.parity   = mm.serial.PARITY_NONE
        self.client.serial.bytesize = 8
        self.client.serial.stopbits = mm.serial.STOPBITS_ONE

    def read_status(self) -> RobotiqModbusRtuStatus:
        values = self.client.read_registers(registeraddress=2000, number_of_registers=3, functioncode=4)
        status = RobotiqModbusRtuStatus.from_registers(values)
        #Each register is 16 bits and therefore contains two unsigned char each
        gripper_status_register, reserved_register            = struct.unpack("BB", values[0].to_bytes(2, "big"))
        fault_status_register, position_request_echo_register = struct.unpack("BB", values[1].to_bytes(2, "big"))
        position_register, current_register                   = struct.unpack("BB", values[2].to_bytes(2, "big"))

        status = GripperStatus

        gripper_fault = GripperFault
        gripper_fault.reactivation_required: bool
        gripper_fault.activation_required: bool
        gripper_fault.overheating: bool
        gripper_fault.undervoltage: bool
        gripper_fault.is_auto_releasing: bool
        gripper_fault.auto_release_completed: bool
        gripper_fault.internal_fault: bool
        gripper_fault.activation_fault: bool
        gripper_fault.overcurrent: bool
        status.fault = gripper_fault

        status.activated: bool
        status.moving: bool
        status.current: float
        status.obj_detected: bool
        status.position: float
        status.goal_position: float
        status.is_reset: bool
        status.is_activating: bool
        status.is_activated: bool

        return status

    

if __name__ == '__main__':
    robotiq_2f85_driver = Robotiq2F85Driver(serial_number='DAK1RLYZ')
    status = robotiq_2f85_driver.read_status()
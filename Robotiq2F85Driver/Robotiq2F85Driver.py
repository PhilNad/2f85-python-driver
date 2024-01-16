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
    #Gripper's temperature has risen above 85C and it needs to cool down.
    overheating: bool
    #There was no communication within the last second
    communication_timeout: bool
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
    #In milliamps
    current: float
    obj_detected: bool
    #In millimeters
    opening: float
    #In millimeters
    goal_opening: float
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

    @property
    def opening(self):
        '''Current opening in millimeters'''
        return self.read_status().opening
    
    @property
    def goal_opening(self):
        '''Goal opening in millimeters'''
        return self.read_status().goal_opening
    
    @property
    def current(self):
        '''Current in milliamps'''
        return self.read_status().current
    
    @property
    def is_reset(self):
        return self.read_status().is_reset
    
    @property
    def is_activating(self):
        return self.read_status().is_activating
    
    @property
    def is_activated(self):
        return self.read_status().is_activated
    
    @property
    def is_moving(self):
        return self.read_status().moving
    
    @property
    def is_activated(self):
        return self.read_status().activated
    
    @property
    def object_detected(self):
        return self.read_status().obj_detected
    
    @property
    def in_fault(self):
        fault = self.read_status().fault
        return fault.reactivation_required or fault.activation_required or fault.overheating or fault.undervoltage or fault.internal_fault or fault.activation_fault or fault.overcurrent
    
    def count_to_opening(self, count:int):
        '''Converts a count to an opening in millimeters'''
        opening = (255 - count) * 0.4
        opening = min(max(opening, 0), 0.085)
        return opening
    
    def opening_to_count(self, opening:float):
        '''Converts an opening in millimeters to a count'''
        count = 255 - opening / 0.4
        count = min(max(count, 0), 255)
        return int(count)
    
    def count_to_speed(self, count:int):
        '''Converts a count to a speed in mm/s
        The speed is between 20-150 mm/s for counts 0-255.
        '''
        speed = count / 255 * (150 - 20) + 20
        speed = min(max(speed, 0), 255)
        return speed
    
    def speed_to_count(self, speed:float):
        '''Converts a speed in mm/s to a count.
        The speed is between 20-150 mm/s for counts 0-255.
        '''
        count = (speed - 20) / (150 - 20) * 255
        count = min(max(count, 0), 255)
        return int(count)
    
    def count_to_force(self, count:int):
        '''Converts a count to a force in N
        The force is between 20-235 N for counts 0-255.
        '''
        force = count / 255 * (235 - 20) + 20
        force = min(max(force, 0), 255)
        return force
    
    def force_to_count(self, force:float):
        '''Converts a force in N to a count
        The force is between 20-235 N for counts 0-255.
        '''
        count = (force - 20) / (235 - 20) * 255
        count = min(max(count, 0), 255)
        return int(count)
    
    def count_to_current(self, count:int):
        '''Converts a count to a current in mA'''
        current = count * 0.1
        return current

    def activate(self, blocking_call:bool=True):
        '''
        Activate the gripper.
        '''
        action_request_register   = 1
        gripper_options1_register = 0
        self.client.write_registers(registeraddress=1000, values=[action_request_register, gripper_options1_register])

        if blocking_call:
            #Read the status until the gripper is activated
            while not self.is_activated:
                pass

    def deactivate(self, blocking_call:bool=True):
        '''
        Deactivate the gripper.
        '''
        action_request_register   = 0
        gripper_options1_register = 0
        self.client.write_registers(registeraddress=1000, values=[action_request_register, gripper_options1_register])

        if blocking_call:
            #Read the status until the gripper is deactivated
            while self.is_activated:
                pass

    def go_to(self, opening:float, speed:float, force:float, blocking_call:bool=True):
        '''
        Move the gripper to the specified opening, speed and force.

        Parameters:
        -----------
        opening : float
            Opening in millimeters. Must be between 0 and 85 mm.
        speed : float
            Speed in mm/s. Must be between 20 and 150 mm/s.
        force : float
            Force in N. Must be between 20 and 235 N.
        '''
        opening_count = self.opening_to_count(opening)
        speed_count   = self.speed_to_count(speed)
        force_count   = self.force_to_count(force)

        #Byte 0
        action_request_register = 2**0 + 2**3
        #Byte 1
        gripper_options1_register = 0
        #Byte 2
        gripper_options2_register = 0
        #Byte 3
        position_request_register = opening_count
        #Byte 4
        speed_register = speed_count
        #Byte 5
        force_register = force_count

        self.client.write_registers(registeraddress=1000, values=[action_request_register, gripper_options1_register, gripper_options2_register, position_request_register, speed_register, force_register])

        if blocking_call:
            #Read the status until the gripper is stopped
            while self.is_moving:
                pass

    def read_status(self) -> GripperStatus:
        values = self.client.read_registers(registeraddress=2000, number_of_registers=3, functioncode=4)
        #Each register is 16 bits and therefore contains two unsigned char each
        gripper_status_register, reserved_register            = struct.unpack("BB", values[0].to_bytes(2, "big"))
        fault_status_register, position_request_echo_register = struct.unpack("BB", values[1].to_bytes(2, "big"))
        position_register, current_register                   = struct.unpack("BB", values[2].to_bytes(2, "big"))

        status = GripperStatus

        gripper_fault = GripperFault
        gripper_fault.reactivation_required = bool(fault_status_register == 0x05)
        gripper_fault.activation_required   = bool(fault_status_register == 0x07)
        gripper_fault.overheating           = bool(fault_status_register == 0x08)
        gripper_fault.undervoltage          = bool(fault_status_register == 0x0A)
        gripper_fault.is_auto_releasing     = bool(fault_status_register == 0x0B)
        gripper_fault.internal_fault        = bool(fault_status_register == 0x0C)
        gripper_fault.activation_fault      = bool(fault_status_register == 0x0D)
        gripper_fault.overcurrent           = bool(fault_status_register == 0x0E)
        gripper_fault.auto_release_completed= bool(fault_status_register == 0x0F)
        status.fault = gripper_fault

        status.activated        = bool(gripper_status_register & 2**0)
        status.moving           = bool(gripper_status_register & 2**3)
        status.is_reset         = not bool(gripper_status_register & 2**4) and not bool(gripper_status_register & 2**5)
        status.is_activating    = bool(gripper_status_register & 2**4) and not bool(gripper_status_register & 2**5)
        status.is_activated     = bool(gripper_status_register & 2**4) and bool(gripper_status_register & 2**5)
        status.obj_detected     = ( bool(gripper_status_register & 2**6) and not bool(gripper_status_register & 2**7) ) or ( not bool(gripper_status_register & 2**6) and bool(gripper_status_register & 2**7) )
        
        status.goal_opening = self.count_to_opening(position_request_echo_register)
        status.current = self.count_to_current(current_register)
        status.opening = self.count_to_opening(position_register)

        return status
    

if __name__ == '__main__':
    robotiq_2f85_driver = Robotiq2F85Driver(serial_number='DAK1RLYZ')
    
from .robotiq_modbus_controller.request import Request as RobotiqModbusRtuRequest
from .robotiq_modbus_controller.status import Status as RobotiqModbusRtuStatus
from .robotiq_modbus_controller.driver import RobotiqModbusRtuDriver
from pathlib import Path
import subprocess


class LinuxFindTTYWithSerialNumber:
    def __init__(self, serial_number):
        self.serial_number = serial_number
        return self.find(serial_number)

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
    
    def get_serial_number(self, tty_device):
        '''
        Get the serial number of a given device.

        Parameters:
        -----------
        tty_device : str
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
    

class Robotiq2F85Driver:
    def __init__(self, serial_number:str):
        self.device_serial_number = serial_number
        self.tty_device = LinuxFindTTYWithSerialNumber(serial_number)
        
        if self.tty_device is None:
            raise Exception('No device with serial number {} found.'.format(serial_number))
        
        self.modebus_driver = RobotiqModbusRtuDriver(self.tty_device)
        self.modebus_driver.connect()
        self.modebus_driver.reset()
        self.modebus_driver.activate()
        self.modebus_driver.move(pos=0, speed=1, force=1)


if __name__ == '__main__':
    robotiq_2f85_driver = Robotiq2F85Driver(serial_number='DAK1RLYZ')
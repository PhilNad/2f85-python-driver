# Copyright 2021 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Union, List
from pymodbus.client import ModbusSerialClient
from pymodbus.client import ModbusTcpClient
from robotiq_modbus_controller.request import Request
from robotiq_modbus_controller.status import Status


class RobotiqModbusDriver:
    INPUT_REGISTER: int = 0
    OUTPUT_REGISTER: int = 0
    NUM_REGISTERS: int = 3

    def __init__(self, client: Union[ModbusSerialClient, ModbusTcpClient]) -> None:
        self._client: Union[ModbusSerialClient, ModbusTcpClient] = client

    def connect(self):
        return self._client.connect()

    def disconnect(self):
        self._client.close()

    def reset(self):
        request: Request = Request()
        self._write_registers(request.registers())

    def activate(self):
        request: Request = Request()
        request.action_request.act = True
        self._write_registers(request.registers())

    def move(self, *, pos: int, speed: int, force: int):
        request: Request = Request()
        request.action_request.act = True
        request.action_request.gto = True
        request.position_request.pr = pos
        request.speed.sp = speed
        request.force.fr = force
        self._write_registers(request.registers())

    def status(self) -> Status:
        response = self._read_registers()
        return Status.from_registers(response)

    def _read_registers(self):
        registers = self._client.read_input_registers(
            self.OUTPUT_REGISTER, self.NUM_REGISTERS
        ).registers
        return registers

    def _write_registers(self, registers: List[int]):
        self._client.write_registers(self.INPUT_REGISTER, registers)


class RobotiqModbusTcpDriver(RobotiqModbusDriver):
    def __init__(self, host: str) -> None:
        client = ModbusTcpClient(host)
        super().__init__(client)


class RobotiqModbusRtuDriver(RobotiqModbusDriver):

    UNIT: int = 9
    INPUT_REGISTER: int = 1000
    OUTPUT_REGISTER: int = 2000
    NUM_REGISTERS: int = 3

    def __init__(self, device) -> None:
        client = ModbusSerialClient(
            method="rtu",
            port=device,
            stopbits=1,
            bytesize=8,
            baudrate=115200,
            timeout=0.2,
        )
        super().__init__(client)

    def _read_registers(self):
        registers = self._client.read_input_registers(
            self.OUTPUT_REGISTER, self.NUM_REGISTERS, unit=self.UNIT
        ).registers
        return registers

    def _write_registers(self, registers: List[int]):
        self._client.write_registers(self.INPUT_REGISTER, registers, unit=self.UNIT)

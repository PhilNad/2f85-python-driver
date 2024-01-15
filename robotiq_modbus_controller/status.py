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

from dataclasses import dataclass
from typing import List
import struct
import numpy as np


@dataclass
class GripperStatus:
    act: bool
    gto: bool
    sta: int
    obj: int


@dataclass
class FaultStatus:
    flt: int
    kflt: int


@dataclass
class PositionRequestEcho:
    pr: int


@dataclass
class Position:
    po: int


@dataclass
class Current:
    cu: int


@dataclass
class Status:
    gripper_status: GripperStatus = GripperStatus(0, 0, 0, 0)
    fault_status: FaultStatus = FaultStatus(0, 0)
    position_request_echo: PositionRequestEcho = PositionRequestEcho(0)
    position: Position = Position(0)
    current: Current = Current(0)

    @classmethod
    def from_registers(cls, registers: List[int]):
        assert len(registers) == 3
        gripper_status, _ = struct.unpack("BB", registers[0].to_bytes(2, "big"))
        fault_status, position_request_echo = struct.unpack(
            "BB", registers[1].to_bytes(2, "big")
        )
        position, current = struct.unpack("BB", registers[2].to_bytes(2, "big"))

        status = Status()
        status.gripper_status.obj = int(format(gripper_status, "08b")[0:2], 2)
        status.gripper_status.sta = int(format(gripper_status, "08b")[2:4], 2)
        status.gripper_status.gto = bool(format(gripper_status, "08b")[4])
        status.gripper_status.act = bool(format(gripper_status, "08b")[7])
        status.fault_status.kflt = int(format(fault_status, "08b")[:4], 2)
        status.fault_status.flt = int(format(fault_status, "08b")[4:], 2)
        status.position_request_echo.pr = position_request_echo
        status.position.po = position
        status.current.cu = current

        return status

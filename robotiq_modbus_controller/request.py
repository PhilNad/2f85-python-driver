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
import numpy as np


@dataclass
class ActionRequest:
    act: bool = False
    gto: bool = False
    atr: bool = False
    ard: bool = False

    def to_byte(self):
        bits = [0, 0, self.ard, self.atr, self.gto, 0, 0, self.act]
        return sum(int(val) * (2 ** idx) for idx, val in enumerate(reversed(bits)))


@dataclass
class GripperOptions:
    lbp: bool = False

    def to_byte(self):
        return int(self.lbp) * 2 ** 4


@dataclass
class PositionRequest:
    pr: int = 0

    def to_byte(self):
        return self.pr


@dataclass
class Speed:
    sp: int = 0

    def to_byte(self):
        return self.sp


@dataclass
class Force:
    fr: int = 0

    def to_byte(self):
        return self.fr


@dataclass
class Request:
    action_request: ActionRequest = ActionRequest(False, False, False, False)
    gripper_options: GripperOptions = GripperOptions(False)
    position_request: PositionRequest = PositionRequest(0)
    speed: Speed = Speed(0)
    force: Force = Force(0)

    def registers(self):
        return [
            (self.action_request.to_byte() << 8) + self.gripper_options.to_byte(),
            self.position_request.to_byte(),
            (self.speed.to_byte() << 8) + self.force.to_byte(),
        ]

from abc import ABC, abstractmethod
from openpilot.selfdrive.car.helpers import convert_carControlSP, convert_to_capnp
from openpilot.sunnypilot.mads.helpers import set_alternative_experience, set_car_specific_params
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.car.card_ext import CarExt


class CarExt(ABC):
  def __init__(self) -> None:
    self.dynamic_experimental_control = False
    self.custom_acc_increments_enabled = False
    self.custom_acc_short_increment = 0.0
    self.custom_acc_long_increment = 0.0
    self.params = None
    self.CP = None
    self.CP_SP = None

  def init_ext(self):
    if self.params is None or self.CP is None or self.CP_SP is None:
      raise ValueError("Params or CarParams not initialized. Are you calling this method too early?")+

    # mads
    set_alternative_experience(self.CP, self.params)
    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Load sunnypilot params
    self.read_sunnypilot_params()

    # Write CarParamsSP for controls
    # convert to pycapnp representation for caching and logging
    self.CP_SP_capnp = convert_to_capnp(self.CP_SP)
    cp_sp_bytes = self.CP_SP_capnp.to_bytes()
    self.params.put("CarParamsSP", cp_sp_bytes)
    self.params.put_nonblocking("CarParamsSPCache", cp_sp_bytes)
    self.params.put_nonblocking("CarParamsSPPersistent", cp_sp_bytes)

  def read_sunnypilot_params(self):
    if self.params is None:
      raise ValueError("Params object is not initialized. Are you calling this method too early?")

    self.dynamic_experimental_control = self.params.get_bool("DynamicExperimentalControl")

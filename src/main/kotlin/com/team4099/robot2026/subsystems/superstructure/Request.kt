package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request

  sealed interface HopperRequest : Request {
    class Idle : HopperRequest

    class OpenLoop(val voltage: ElectricalPotential) : HopperRequest
  }
}

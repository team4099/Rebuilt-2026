package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request

  sealed interface ClimbRequest : Request {
    class TargetingPosition(val position: Length) : ClimbRequest

    class OpenLoop(val voltage: ElectricalPotential) : ClimbRequest
  }
}

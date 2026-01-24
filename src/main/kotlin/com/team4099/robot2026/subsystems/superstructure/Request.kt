package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request

  sealed interface ShooterRequest : Request {
    class OpenLoop(val shooterVoltage: ElectricalPotential) : ShooterRequest

    class TargetVelocity(val targetVelocity: AngularVelocity) : ShooterRequest

    class Idle() : ShooterRequest
  }
}

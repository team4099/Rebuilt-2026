package com.team4099.robot2026.subsystems.superstructure

import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential

sealed interface Request {
  sealed interface SuperstructureRequest : Request {
    class Idle() : SuperstructureRequest

    class ExtendClimb() : SuperstructureRequest

    class RetractClimb() : SuperstructureRequest

    class PrepScore() : SuperstructureRequest

    class Score() : SuperstructureRequest

    class Intake() : SuperstructureRequest

    class Eject() : SuperstructureRequest
  }

  sealed interface ClimbRequest : Request {
    class TargetingPosition(val position: Length) : ClimbRequest

    class OpenLoop(val voltage: ElectricalPotential) : ClimbRequest
  }

  sealed interface HopperRequest : Request {
    class Idle : HopperRequest

    class OpenLoop(val voltage: ElectricalPotential) : HopperRequest
  }

  sealed interface FeederRequest : Request {
    class Idle : FeederRequest

    class OpenLoop(val voltage: ElectricalPotential) : FeederRequest
  }

  sealed interface RollersRequest : Request {
    class OpenLoop(val voltage: ElectricalPotential) : RollersRequest
  }

  sealed interface IntakeRequest : Request {
    class OpenLoop(val pivotVoltage: ElectricalPotential) : IntakeRequest
    class TargetingPosition(val pivotPosition: Angle) : IntakeRequest
    class ZeroPivot : IntakeRequest
  }

  sealed interface ShooterRequest : Request {
    class OpenLoop(val shooterVoltage: ElectricalPotential) : ShooterRequest

    class TargetVelocity(val targetVelocity: AngularVelocity) : ShooterRequest

    class Idle : ShooterRequest
  }
}

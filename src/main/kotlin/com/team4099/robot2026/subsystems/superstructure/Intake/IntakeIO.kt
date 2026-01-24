package com.team4099.robot2026.subsystems.superstructure.Intake

import org.ironmaple.simulation.IntakeSimulation
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

interface IntakeIO {
  val intakeSimulation: IntakeSimulation?

  class IntakeIOInputs : LoggableInputs {
    var position = 0.radians
    var intakeAppliedVoltage = 0.volts
    var intakeTemperature = 0.0.celsius
    var intakeStatorCurrent = 0.amps
    var intakeSupplyCurrent = 0.amps
    var velocity = 0.0.rotations.perMinute

    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("intakeTemperatureCelsius", intakeTemperature.inCelsius)
      table?.put("intakeAppliedVolts", intakeAppliedVoltage.inVolts)
      table?.put("intakePositionRad", position.inRadians)
      table?.put("intakeStatorCurrentAmps", intakeStatorCurrent.inAmperes)
      table?.put("intakeSupplyCurrentAmps", intakeSupplyCurrent.inAmperes)
      table?.put("IntakeVelocity", velocity.inRotationsPerMinute)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("intakeTemperatureCelsius", intakeTemperature.inCelsius)?.let {
        intakeTemperature = it.celsius
      }
      table?.get("intakeVelocity", velocity.inRotationsPerMinute)?.let {
        velocity = it.rotations.perMinute
      }
      table?.get("intakeAppliedVolts", intakeAppliedVoltage.inVolts)?.let {
        intakeAppliedVoltage = it.volts
      }
      table?.get("intakePositionRad", position.inRadians)?.let { position = it.radians }
      table?.get("intakeStatorCurrentAmps", intakeStatorCurrent.inAmperes)?.let {
        intakeStatorCurrent = it.amps
      }
      table?.get("intakeSupplyCurrentAmps", intakeSupplyCurrent.inAmperes)?.let {
        intakeSupplyCurrent = it.amps
      }
    }
  }

  fun zeroPivot() {}

  fun updateInputs(inputs: IntakeIOInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setPosition(position: Angle) {}

  fun configPID(
      kP: ProportionalGain<Radian, Volt>,
      kI: IntegralGain<Radian, Volt>,
      kD: DerivativeGain<Radian, Volt>,
  ) {}

  fun configFF(
      kG: ElectricalPotential,
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {}

  fun setBrakeMode(brake: Boolean) {}
}

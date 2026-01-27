package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute

interface IntakeRollersIO {
  class RollerInputs : LoggableInputs {
    var rollerVelocity = 0.rotations.perMinute
    var rollerAcceleration = 0.rotations.perMinute.perMinute
    var rollerAppliedVoltage = 0.volts
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTemperature = 0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("rollerTemperatureCelsius", rollerTemperature.inCelsius)
      table?.put("rollerAppliedVolts", rollerAppliedVoltage.inVolts)
      table?.put("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put("rollerAccelerationRPMPM", rollerAcceleration.inRotationsPerMinutePerMinute)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("rollerTemperatureCelsius", rollerTemperature.inCelsius)?.let {
        rollerTemperature = it.celsius
      }
      table?.get("rollerAppliedVolts", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.get("rollerVelocityRPM", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.rotations.perMinute
      }
      table?.get("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.get("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table?.get("rollerAccelerationRPMPM", rollerAcceleration.inRotationsPerMinutePerMinute)?.let {
        rollerAcceleration = it.rotations.perMinute.perMinute
      }
    }
  }

  fun updateInputs(inputs: RollerInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}
}

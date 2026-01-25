package com.team4099.robot2025.subsystems.superstructure.Intake.Rollers

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perSecond

interface RollersIO {
  class RollerInputs : LoggableInputs {
    var rollerVelocity = 0.degrees.perSecond
    var rollerAcceleration = 0.degrees.perSecond.perSecond
    var rollerAppliedVoltage = 0.volts
    var rollerDutyCycle = 0.volts
    var rollerStatorCurrent = 0.amps
    var rollerSupplyCurrent = 0.amps
    var rollerTemperature = 0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("rollerTemperatureCelsius", rollerTemperature.inCelsius)
      table?.put("rollerAppliedVolts", rollerAppliedVoltage.inVolts)
      table?.put("rollerVelocityDegreesPerSecond", rollerVelocity.inDegreesPerSecond)
      table?.put("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)
      table?.put(
          "rollerAccelerationDegreesPerSecondPerSecond",
          rollerAcceleration.inDegreesPerSecondPerSecond)
      table?.put("rollerDutyCycle", rollerDutyCycle.inVolts)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("rollerTemperatureCelsius", rollerTemperature.inCelsius)?.let {
        rollerTemperature = it.celsius
      }
      table?.get("rollerAppliedVolts", rollerAppliedVoltage.inVolts)?.let {
        rollerAppliedVoltage = it.volts
      }
      table?.get("rollerVelocityDegreesPerSecond", rollerVelocity.inRotationsPerMinute)?.let {
        rollerVelocity = it.degrees.perSecond
      }
      table?.get("rollerStatorCurrentAmps", rollerStatorCurrent.inAmperes)?.let {
        rollerStatorCurrent = it.amps
      }
      table?.get("rollerSupplyCurrentAmps", rollerSupplyCurrent.inAmperes)?.let {
        rollerSupplyCurrent = it.amps
      }
      table
          ?.get(
              "rollerAccelerationDegreesPerSecondPerSecond",
              rollerAcceleration.inDegreesPerSecondPerSecond)
          ?.let { rollerAcceleration = it.degrees.perSecond.perSecond }
      table?.get("rollerDutyCycle", rollerDutyCycle.inVolts)?.let { rollerDutyCycle = it.volts }
    }
  }

  fun updateInputs(inputs: RollerInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}
}

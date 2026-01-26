package com.team4099.robot2026.subsystems.climb

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

interface ClimbIO {
  class ClimbInputs : LoggableInputs {
    var climbPosition = 0.0.inches
    var appliedVoltage = 0.0.volts
    var temperature = 0.0.celsius
    var supplyCurrent = 0.0.amps
    var statorCurrent = 0.0.amps
    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("climbPositionInches", climbPosition.inInches)
      table?.put("climbAppliedVoltage", appliedVoltage.inVolts)
      table?.put("climbTempCelsius", temperature.inCelsius)
      table?.put("climbSupplyCurrent", supplyCurrent.inAmperes)
      table?.put("climbStatorCurrent", statorCurrent.inAmperes)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("climbPositionInches", climbPosition.inInches)?.let { climbPosition = it.inches }
      table?.get("climbAppliedVoltage", appliedVoltage.inVolts)?.let { appliedVoltage = it.volts }
      table?.get("climbTempCelsius", temperature.inCelsius)?.let { temperature = it.celsius }
      table?.get("climbSupplyCurrent", supplyCurrent.inAmperes)?.let { supplyCurrent = it.amps }
      table?.get("climbStatorCurrent", statorCurrent.inAmperes)?.let { statorCurrent = it.amps }
    }
  }

  fun updateInputs(inputs: ClimbInputs) {}

  fun setPosition(position: Length) {}

  fun setVoltage(targetVoltage: ElectricalPotential) {}

  fun zeroEncoder() {}

  fun configPID(
      kP: ProportionalGain<Meter, Volt>,
      kI: IntegralGain<Meter, Volt>,
      kD: DerivativeGain<Meter, Volt>
  ) {}

  fun configFF(kS: ElectricalPotential, kG: ElectricalPotential) {}
}

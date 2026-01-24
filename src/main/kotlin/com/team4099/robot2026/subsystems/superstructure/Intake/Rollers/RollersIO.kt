package com.team4099.robot2026.subsystems.superstructure.Intake.Rollers

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
import org.team4099.lib.units.perSecond

interface RollersIO {
  class RollerInputs : LoggableInputs {
    var rollerAppliedVoltage = 0.0.volts
    var rollerVelocity = 0.0.degrees.perSecond
    var rollerSupplyCurrent = 0.0.amps
    var rollerStatorCurrent = 0.0.amps
    var rollerTemp = 0.0.celsius

    var isSimulating = false

    override fun toLog(table: LogTable) {
      table.put("rollerAppliedVoltage", rollerAppliedVoltage.inVolts)
      table.put("rollerVelocity", rollerVelocity.inDegreesPerSecond)
      table.put("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes)
      table.put("rollerStatorCurrent", rollerStatorCurrent.inAmperes)
      table.put("rollerTemp", rollerTemp.inCelsius)
      table.put("isSimulating", isSimulating)
    }

    override fun fromLog(table: LogTable) {
      table.get("rollerAppliedVoltage", rollerAppliedVoltage.inVolts).let {
        rollerAppliedVoltage = it.volts
      }
      table.get("rollerVelocity", rollerVelocity.inDegreesPerSecond).let {
        rollerVelocity = it.degrees.perSecond
      }
      table.get("rollerSupplyCurrent", rollerSupplyCurrent.inAmperes).let {
        rollerSupplyCurrent = it.amps
      }
      table.get("rollerStatorCurrent", rollerStatorCurrent.inAmperes).let {
        rollerStatorCurrent = it.amps
      }
      table.get("rollerAppliedVoltage", rollerAppliedVoltage.inVolts).let {
        rollerAppliedVoltage = it.volts
      }
      table.get("isSimulating", isSimulating)
    }
  }

  fun updateInputs(inputs: RollerInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}
}

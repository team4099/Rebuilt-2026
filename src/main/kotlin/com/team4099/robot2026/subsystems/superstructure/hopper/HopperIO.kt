package com.team4099.robot2026.subsystems.superstructure.hopper

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.*
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface HopperIO {

  fun updateInputs(inputs: HopperIOInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setBrakeMode(brake: Boolean) {}

  class HopperIOInputs : LoggableInputs {
    // Hopper Inputs
    var hopperAngularVelocity = 0.0.degrees.perSecond
    var hopperAcceleration = 0.0.degrees.perSecond.perSecond
    var hopperAppliedVoltage = 0.0.volts
    var hopperStatorCurrent = 0.0.amps
    var hopperSupplyCurrent = 0.0.amps
    var hopperTemp = 0.0.celsius

    override fun toLog(table: LogTable) {
      table.put("hopperVelocityPerSecond", hopperAngularVelocity.inDegreesPerSecond)
      table.put("hopperAccelerationDPSPS", hopperAcceleration.inDegreesPerSecondPerSecond)
      table.put("hopperAppliedVoltage", hopperAppliedVoltage.inVolts)
      table.put("hopperStatorCurrent", hopperStatorCurrent.inAmperes)
      table.put("hopperSupplyCurrent", hopperSupplyCurrent.inAmperes)
      table.put("hopperTemp", hopperTemp.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      table.get("hopperVelocityPerSecond", hopperAngularVelocity.inDegreesPerSecond).let {
        hopperAngularVelocity = it.degrees.perSecond
      }
      table.get("hopperAcceleration", hopperAcceleration.inDegreesPerSecondPerSecond).let {
        hopperAcceleration = it.degrees.perSecond.perSecond
      }
      table.get("hopperAppliedVoltage", hopperAppliedVoltage.inVolts).let {
        hopperAppliedVoltage = it.volts // idk if this is right
      }
      table.get("hopperStatorCurrent", hopperStatorCurrent.inAmperes).let {
        hopperStatorCurrent = it.amps
      }
      table.get("hopperSupplyCurrent", hopperSupplyCurrent.inAmperes).let {
        hopperSupplyCurrent = it.amps
      }
      table.get("hopperTemp", hopperTemp.inCelsius).let { hopperTemp = it.celsius }
    }
  }
}

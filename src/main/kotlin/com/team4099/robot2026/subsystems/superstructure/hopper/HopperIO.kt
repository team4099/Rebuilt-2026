package com.team4099.robot2026.subsystems.superstructure.hopper

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
import org.team4099.lib.units.perMinute

interface HopperIO {
  class HopperIOInputs : LoggableInputs {
    // Hopper Inputs
    var hopperAngularVelocity = 0.0.rotations.perMinute
    var hopperAngularAcceleration = 0.0.rotations.perMinute
    var hopperAppliedVoltage = 0.0.volts
    var hopperStatorCurrent = 0.0.amps
    var hopperSupplyCurrent = 0.0.amps
    var hopperTemp = 0.0.celsius

    override fun toLog(table: LogTable) {
      table.put("hopperVelocityRPM", hopperAngularVelocity.inRotationsPerMinute)
      table.put("hopperAccelerationRPM", hopperAngularAcceleration.inRotationsPerMinute)
      table.put("hopperAppliedVoltage", hopperAppliedVoltage.inVolts)
      table.put("hopperStatorCurrent", hopperStatorCurrent.inAmperes)
      table.put("hopperSupplyCurrent", hopperSupplyCurrent.inAmperes)
      table.put("hopperTemp", hopperTemp.inCelsius)
    }

    override fun fromLog(table: LogTable) {
      table.get("HopperVelocityRPM", hopperAngularVelocity.inRotationsPerMinute).let {
        hopperAngularVelocity = it.rotations.perMinute
      }
      table.get("hopperAccelerationRPM", hopperAngularAcceleration.inRotationsPerMinute).let {
        hopperAngularAcceleration = it.rotations.perMinute
      }
      table.get("hopperAppliedVoltage", hopperAppliedVoltage.inVolts).let {
        hopperAppliedVoltage = it.volts
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

  fun updateInputs(inputs: HopperIOInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setBrakeMode(brake: Boolean) {}
}

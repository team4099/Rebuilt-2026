package com.team4099.robot2025.subsystems.Feeders

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

interface FeederIO {
  class FeederIOInputs : LoggableInputs {
    // Feeder Inputs
    var feederVelocity = 0.rotations.perMinute
    var feederAcceleration = 0.rotations.perMinute.perMinute
    var feederAppliedVoltage = 0.0.volts
    var feederStatorCurrent = 0.0.amps
    var feederSupplyCurrent = 0.0.amps
    var feederTemp = 0.0.celsius
    var isSimulating = false

    override fun toLog(table: LogTable?) {
      table?.put("FeederVelocityRPM", feederVelocity.inRotationsPerMinute)
      table?.put("FeederAccelerationRPMPM", feederAcceleration.inRotationsPerMinutePerMinute)
      table?.put("FeederAppliedVoltage", feederAppliedVoltage.inVolts)
      table?.put("FeederStatorCurrentAmps", feederStatorCurrent.inAmperes)
      table?.put("FeederSupplyCurrentAmps", feederSupplyCurrent.inAmperes)
      table?.put("FeederTempCelsius", feederTemp.inCelsius)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("FeederVelocityRPM", feederVelocity.inRotationsPerMinute)?.let {
        feederVelocity = it.rotations.perMinute
      }
      table?.get("FeederAccelerationRPMPM", feederAcceleration.inRotationsPerMinutePerMinute)?.let {
        feederAcceleration = it.rotations.perMinute.perMinute
      }
      table?.get("FeederAppliedVoltage", feederAppliedVoltage.inVolts)?.let {
        feederAppliedVoltage = it.volts
      }
      table?.get("FeederStatorCurrentAmps", feederStatorCurrent.inAmperes)?.let {
        feederStatorCurrent = it.amps
      }
      table?.get("FeederSupplyCurrentAmps", feederSupplyCurrent.inAmperes)?.let {
        feederSupplyCurrent = it.amps
      }
      table?.get("FeederTempCelsius", feederTemp.inCelsius)?.let { feederTemp = it.celsius }
    }
  }

  fun updateInputs(inputs: FeederIOInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun setBrakeMode(brake: Boolean) {}
}
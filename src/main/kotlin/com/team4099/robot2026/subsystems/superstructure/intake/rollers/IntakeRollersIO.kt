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
    var leaderVelocity = 0.rotations.perMinute
    var leaderAcceleration = 0.rotations.perMinute.perMinute
    var leaderAppliedVoltage = 0.volts
    var leaderStatorCurrent = 0.amps
    var leaderSupplyCurrent = 0.amps
    var leaderTemperature = 0.celsius

    var followerVelocity = 0.rotations.perMinute
    var followerAcceleration = 0.rotations.perMinute.perMinute
    var followerAppliedVoltage = 0.volts
    var followerStatorCurrent = 0.amps
    var followerSupplyCurrent = 0.amps
    var followerTemperature = 0.celsius

    override fun toLog(table: LogTable?) {
      table?.put("rollerTemperatureCelsius", leaderTemperature.inCelsius)
      table?.put("rollerAppliedVolts", leaderAppliedVoltage.inVolts)
      table?.put("rollerVelocityRPM", leaderVelocity.inRotationsPerMinute)
      table?.put("rollerStatorCurrentAmps", leaderStatorCurrent.inAmperes)
      table?.put("rollerSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)
      table?.put("rollerAccelerationRPMPM", leaderAcceleration.inRotationsPerMinutePerMinute)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("rollerTemperatureCelsius", leaderTemperature.inCelsius)?.let {
        leaderTemperature = it.celsius
      }
      table?.get("rollerAppliedVolts", leaderAppliedVoltage.inVolts)?.let {
        leaderAppliedVoltage = it.volts
      }
      table?.get("rollerVelocityRPM", leaderVelocity.inRotationsPerMinute)?.let {
        leaderVelocity = it.rotations.perMinute
      }
      table?.get("rollerStatorCurrentAmps", leaderStatorCurrent.inAmperes)?.let {
        leaderStatorCurrent = it.amps
      }
      table?.get("rollerSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)?.let {
        leaderSupplyCurrent = it.amps
      }
      table?.get("rollerAccelerationRPMPM", leaderAcceleration.inRotationsPerMinutePerMinute)?.let {
        leaderAcceleration = it.rotations.perMinute.perMinute
      }
    }
  }

  fun updateInputs(inputs: RollerInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}
}

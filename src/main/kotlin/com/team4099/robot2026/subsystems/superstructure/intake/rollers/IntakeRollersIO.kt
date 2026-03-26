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
      table?.put("leaderTemperatureCelsius", leaderTemperature.inCelsius)
      table?.put("leaderAppliedVolts", leaderAppliedVoltage.inVolts)
      table?.put("leaderVelocityRPM", leaderVelocity.inRotationsPerMinute)
      table?.put("leaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)
      table?.put("leaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)
      table?.put("leaderAccelerationRPMPM", leaderAcceleration.inRotationsPerMinutePerMinute)
      table?.put("followerTemperatureCelsius", followerTemperature.inCelsius)
      table?.put("followerAppliedVolts", followerAppliedVoltage.inVolts)
      table?.put("followerVelocityRPM", followerVelocity.inRotationsPerMinute)
      table?.put("followerStatorCurrentAmps", followerStatorCurrent.inAmperes)
      table?.put("followerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)
      table?.put("followerAccelerationRPMPM", followerAcceleration.inRotationsPerMinutePerMinute)
    }

    override fun fromLog(table: LogTable?) {
      table?.get("leaderTemperatureCelsius", leaderTemperature.inCelsius)?.let {
        leaderTemperature = it.celsius
      }
      table?.get("leaderAppliedVolts", leaderAppliedVoltage.inVolts)?.let {
        leaderAppliedVoltage = it.volts
      }
      table?.get("leaderVelocityRPM", leaderVelocity.inRotationsPerMinute)?.let {
        leaderVelocity = it.rotations.perMinute
      }
      table?.get("leaderStatorCurrentAmps", leaderStatorCurrent.inAmperes)?.let {
        leaderStatorCurrent = it.amps
      }
      table?.get("leaderSupplyCurrentAmps", leaderSupplyCurrent.inAmperes)?.let {
        leaderSupplyCurrent = it.amps
      }
      table?.get("leaderAccelerationRPMPM", leaderAcceleration.inRotationsPerMinutePerMinute)?.let {
        leaderAcceleration = it.rotations.perMinute.perMinute
      }
      table?.get("followerTemperatureCelsius", followerTemperature.inCelsius)?.let {
        followerTemperature = it.celsius
      }
      table?.get("followerAppliedVolts", followerAppliedVoltage.inVolts)?.let {
        followerAppliedVoltage = it.volts
      }
      table?.get("followerVelocityRPM", followerVelocity.inRotationsPerMinute)?.let {
        followerVelocity = it.rotations.perMinute
      }
      table?.get("followerStatorCurrentAmps", followerStatorCurrent.inAmperes)?.let {
        followerStatorCurrent = it.amps
      }
      table?.get("followerSupplyCurrentAmps", followerSupplyCurrent.inAmperes)?.let {
        followerSupplyCurrent = it.amps
      }
      table
          ?.get("followerAccelerationRPMPM", followerAcceleration.inRotationsPerMinutePerMinute)
          ?.let { followerAcceleration = it.rotations.perMinute.perMinute }
    }
  }

  fun updateInputs(inputs: RollerInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}
}

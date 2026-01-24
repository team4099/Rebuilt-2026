package com.team4099.robot2026.subsystems.superstructure.shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

interface ShooterIO {
  class ShooterInputs : LoggableInputs {
    // leader variables
    var shooterLeaderVoltage = 0.0.volts
    var shooterLeaderSupplyCurrent = 0.0.amps
    var shooterLeaderStatorCurrent = 0.0.amps
    var shooterLeaderTemperature = 0.0.celsius
    var shooterLeaderVelocity = 0.0.degrees.perSecond
    var shooterLeaderAcceleration = 0.0.degrees.perSecond.perSecond

    // follower variables
    var shooterFollowerVoltage = 0.0.volts
    var shooterFollowerSupplyCurrent = 0.0.amps
    var shooterFollowerStatorCurrent = 0.0.amps
    var shooterFollowerTemperature = 0.0.celsius
    var shooterFollowerVelocity = 0.0.degrees.perSecond
    var shooterFollowerAcceleration = 0.0.degrees.perSecond.perSecond

    override fun toLog(table: LogTable?) {

      // leader
      table?.put("ShooterLeaderVoltage", shooterLeaderVoltage.inVolts)
      table?.put("ShooterLeaderSupplyCurrent", shooterLeaderSupplyCurrent.inAmperes)
      table?.put("ShooterLeaderStatorCurrent", shooterLeaderStatorCurrent.inAmperes)
      table?.put("ShooterLeaderTemperature", shooterLeaderTemperature.inCelsius)
      table?.put("ShooterLeaderVelocityRPM", shooterLeaderVelocity.inRotationsPerMinute)
      table?.put(
          "ShooterLeaderAccelerationRPM", shooterLeaderAcceleration.inRotationsPerMinutePerMinute)
      // follower
      table?.put("ShooterFollowerVoltage", shooterFollowerVoltage.inVolts)
      table?.put("ShooterFollowerSupplyCurrent", shooterFollowerSupplyCurrent.inAmperes)
      table?.put("ShooterFollowerStatorCurrent", shooterFollowerStatorCurrent.inAmperes)
      table?.put("ShooterFollowerTemperature", shooterFollowerTemperature.inCelsius)
      table?.put("ShooterFollowerVelocityRPM", shooterFollowerVelocity.inRotationsPerMinute)
      table?.put(
          "ShooterFollowerAccelerationRPM",
          shooterFollowerAcceleration.inRotationsPerMinutePerMinute)
    }

    override fun fromLog(table: LogTable) {

      // leader
      table.get("ShooterLeaderVoltage", shooterLeaderVoltage.inVolts).let {
        shooterLeaderVoltage = it.volts
      }
      table.get("ShooterLeaderSupplyCurrent", shooterLeaderSupplyCurrent.inAmperes).let {
        shooterLeaderSupplyCurrent = it.amps
      }
      table.get("ShooterLeaderStatorCurrent", shooterLeaderStatorCurrent.inAmperes).let {
        shooterLeaderStatorCurrent = it.amps
      }
      table.get("ShooterLeaderTemperatureCurrent", shooterLeaderTemperature.inCelsius).let {
        shooterLeaderTemperature = it.celsius
      }
      table.get("ShooterLeaderVelocityRPM", shooterLeaderVelocity.inRotationsPerMinute).let {
        shooterLeaderVelocity = it.rotations.perMinute
      }
      table
          .get(
              "ShooterLeaderAccelerationRPM",
              shooterLeaderAcceleration.inRotationsPerMinutePerMinute)
          .let { shooterLeaderAcceleration = it.rotations.perMinute.perMinute }
      // follower
      table.get("ShooterFollowerVoltage", shooterFollowerVoltage.inVolts).let {
        shooterFollowerVoltage = it.volts
      }
      table.get("ShooterFollowerSupplyCurrent", shooterFollowerSupplyCurrent.inAmperes).let {
        shooterFollowerSupplyCurrent = it.amps
      }
      table.get("ShooterFollowerStatorCurrent", shooterFollowerStatorCurrent.inAmperes).let {
        shooterFollowerStatorCurrent = it.amps
      }
      table.get("ShooterFollowerTemperatureCurrent", shooterFollowerTemperature.inCelsius).let {
        shooterFollowerTemperature = it.celsius
      }
      table.get("ShooterFollowerVelocityRPM", shooterFollowerVelocity.inRotationsPerMinute).let {
        shooterFollowerVelocity = it.rotations.perMinute
      }
      table
          .get(
              "ShooterFollowerAccelerationRPM",
              shooterFollowerAcceleration.inRotationsPerMinutePerMinute)
          .let { shooterFollowerAcceleration = it.rotations.perMinute.perMinute }
    }
  }

  fun updateInputs(inputs: ShooterInputs) {}

  fun setVoltage(voltage: ElectricalPotential) {}

  fun configurePID(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {}

  fun configureFF(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>,
  ) {}

  fun setVelocity(velocity: AngularVelocity) {}
}

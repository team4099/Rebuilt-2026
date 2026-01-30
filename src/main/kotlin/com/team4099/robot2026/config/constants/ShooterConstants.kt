package com.team4099.robot2026.config.constants

import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object ShooterConstants {
  val SHOOTER_HEIGHT = 20.085331.inches
  val SHOOTER_ANGLE = 60.degrees
  val SHOOTER_OFFSET = Transform2d(Translation2d(6.852263.inches, -(4.780512 + 0.75/2).inches), 0.radians)

  val GEAR_RATIO: Double = 24.0 / 24.0
  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 40.0.amps
  val VOLTAGE_COMPENSATION = 12.0.volts
  val MOMENT_OF_INERTIA = 0.002341.kilo.grams.meterSquared

  val MAX_ACCELERATION: AngularAcceleration = 200.rotations.perSecond.perSecond

  val SHOOTER_TOLERANCE = 75.0.rotations.perMinute

  object VELOCITIES {
    val MINIMUM_LAUNCH_VELOCITY = 1200.0.rotations.perMinute
    val IDLE_VELOCITY = MINIMUM_LAUNCH_VELOCITY
  }

  object PID {
    val REAL_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.0.volts / 1.0.degrees.perSecond
    val REAL_KI: IntegralGain<Velocity<Radian>, Volt> =
        0.05.volts / (1.0.radians.perSecond * 1.0.seconds)
    val REAL_KD: DerivativeGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.degrees.perSecond / 1.0.seconds)

    // SYS ID
    val REAL_KS: StaticFeedforward<Volt> = 0.17726.volts
    val REAL_KV: VelocityFeedforward<Radian, Volt> = 0.1195.volts / 1.radians.perSecond
    val REAL_KA: AccelerationFeedforward<Radian, Volt> =
        0.01765.volts / 1.radians.perSecond.perSecond

    val SIM_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.02.volts / 1.0.degrees.perSecond
    val SIM_KI: IntegralGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute * 0.0.seconds)
    val SIM_KD: DerivativeGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute.perSecond)

    val SIM_KS: StaticFeedforward<Volt> = 0.0.volts
    val SIM_KV: VelocityFeedforward<Radian, Volt> = (1.0 / 3000.0).volts / 1.degrees.perSecond
    val SIM_KA: AccelerationFeedforward<Radian, Volt> = 0.0.volts / 1.degrees.perSecond.perSecond
  }
}

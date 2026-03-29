package com.team4099.robot2026.config.constants

import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.geometry.Translation3d
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

object HopperConstants {
  const val GEAR_RATIO: Double = 1.0 / 9.0

  val STATOR_CURRENT_LIMIT = 60.amps
  val SUPPLY_CURRENT_LIMIT = 60.amps

  val VOLTAGE_COMPENSATION = 12.volts

  val MOMENT_OF_INERTIA = 0.016973.kilo.grams.meterSquared
  val MAX_ACCELERATION: AngularAcceleration = 1000.rotations.perSecond.perSecond

  val SCORE_VOLTAGE = 12.volts
  val UNJAM_VOLTAGE = -12.volts

  const val SIM_X_CAPACITY = 5
  const val SIM_Y_CAPACITY = 4
  val FUEL_DIAMETER = 5.9.inches

  val originTBottomRight =
      Transform3d(Translation3d(-10.inches - 12.inches, -9.5.inches, 0.inches), Rotation3d())
  val HOPPER_FLOOR_START = 4.417879.inches + FUEL_DIAMETER / 2.0

  object VELOCITIES {
    val IDLE_VELOCITY = 0.rotations.perSecond
    val SCORE_VELOCITY = 9.rotations.perSecond
  }

  object PID {
    val REAL_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.volts / 1.0.radians.perSecond
    val REAL_KI: IntegralGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.radians.perSecond * 1.0.seconds)
    val REAL_KD: DerivativeGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.radians.perSecond / 1.0.seconds)

    val REAL_KS: StaticFeedforward<Volt> = 0.2252.volts
    val REAL_KV: VelocityFeedforward<Radian, Volt> = 0.12649.volts / 1.radians.perSecond
    val REAL_KA: AccelerationFeedforward<Radian, Volt> =
        0.0051489.volts / 1.radians.perSecond.perSecond

    val SIM_KP: ProportionalGain<Velocity<Radian>, Volt> = 0.0.volts / 1.0.degrees.perSecond
    val SIM_KI: IntegralGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute * 0.0.seconds)
    val SIM_KD: DerivativeGain<Velocity<Radian>, Volt> =
        0.0.volts / (1.0.rotations.perMinute.perSecond)

    val SIM_KS: StaticFeedforward<Volt> = 0.0.volts
    val SIM_KV: VelocityFeedforward<Radian, Volt> = 0.volts / 1.degrees.perSecond
    val SIM_KA: AccelerationFeedforward<Radian, Volt> = 0.0.volts / 1.degrees.perSecond.perSecond
  }
}

package com.team4099.robot2026.config.constants

import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
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
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object FeederConstants {
  const val GEAR_RATIO = 12.0 / 24.0

  val STATOR_CURRENT_LIMIT = 50.0.amps
  val SUPPLY_CURRENT_LIMIT = 40.0.amps

  val MOMENT_OF_INERTIA = 0.35.grams.meterSquared
  val VOLTAGE_COMPENSATION = 12.0.volts

  val IDLE_VOLTAGE = if (Constants.Tuning.TUNING_MODE) 0.volts else -.5.volts
  val SCORE_VOLTAGE = 5.0.volts
  val SCORE_VELOCITY = 10.rotations.perSecond

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

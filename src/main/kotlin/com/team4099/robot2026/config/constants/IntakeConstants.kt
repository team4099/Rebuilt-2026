package com.team4099.robot2026.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeConstants {
  const val GEAR_RATIO = (12.0 / 36.0) * (18.0 / 54.0) * (12.0 / 36.0)

  val INTAKE_TOLERANCE = 2.0.degrees

  val PIVOT_INERTIA = 1.0.grams.meterSquared
  val PIVOT_LENGTH = 1.0.inches
  val PIVOT_MAX_ANGLE = 120.degrees
  val PIVOT_MIN_ANGLE = (-50).degrees

  val STATOR_CURRENT_LIMIT = 55.amps
  val SUPPLY_CURRENT_LIMIT = 40.amps

  val VOLTAGE_COMPENSATION = 12.0.volts

  val MAX_VELOCITY = 1000.rotations.perSecond
  val MAX_ACCELERATION = 1000.rotations.perSecond.perSecond
  val MAX_JERK = 3000.rotations.perSecond.perSecond.perSecond

  val SIM_VELOCITY = 400.degrees.perSecond
  val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

  val LENGTH_EXTENDED = 0.0.inches

  object ANGLES {
    val INTAKE_ANGLE = PIVOT_MIN_ANGLE
    val STOW_ANGLE = PIVOT_MAX_ANGLE
    val IDLE_ANGLE = 10.degrees
    val EJECT_ANGLE = STOW_ANGLE
  }

  object PID {
    // PID Constants
    val REAL_PIVOT_KP: ProportionalGain<Radian, Volt> = 32.5.volts / 1.0.radians
    val REAL_PIVOT_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.radians * 1.0.seconds)
    val REAL_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.radians.perSecond

    val SIM_PIVOT_KP: ProportionalGain<Radian, Volt> = .1.volts / 1.0.radians
    val SIM_PIVOT_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.radians * 1.0.seconds)
    val SIM_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.radians.perSecond

    val PIVOT_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    val PIVOT_KV = 0.0.volts / 1.0.radians.perSecond
    val PIVOT_KG = 0.2.volts
    val PIVOT_KS = 0.23.volts

    val SIM_PIVOT_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
    val SIM_PIVOT_KV = 0.0.volts / 1.0.radians.perSecond
    val SIM_PIVOT_KG = 0.0.volts
    val SIM_PIVOT_KS = 0.0.volts
  }
}

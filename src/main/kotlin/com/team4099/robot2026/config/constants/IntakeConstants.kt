package com.team4099.robot2025.config.constants

import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.pounds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeConstants {
  val ZERO_OFFSET = ANGLES.STOW_ANGLE
  val GEAR_RATIO = (8.0 / 40.0) * (16.0 / 50.0) * (16.0 / 44.0)

  val INTAKE_TOLERANCE = 1.0.degrees

  val PIVOT_INERTIA = 0.0.grams.meterSquared
  val PIVOT_LENGTH = 0.0.inches
  val PIVOT_MAX_ANGLE = 120.0.degrees
  val PIVOT_MIN_ANGLE = 0.0.degrees

  // Feedforward Constants
  val PIVOT_KA = 0.0.volts / 1.0.radians.perSecond.perSecond
  val PIVOT_KV = 0.0.volts / 1.0.radians.perSecond
  val PIVOT_KG = 0.3.volts
  val PIVOT_KS = 0.1.volts

  val STATOR_CURRENT_LIMIT = 60.amps
  val SUPPLY_CURRENT_LIMIT = 60.amps

  val VOLTAGE_COMPENSATION = 12.0.volts

  val MAX_VELOCITY = 125.degrees.perSecond
  val MAX_ACCELERATION = 125.degrees.perSecond.perSecond

  val SIM_VELOCITY = 400.degrees.perSecond
  val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

  val FORCE_DOWN_VOLTAGE = (-3).volts

  val LENGTH_EXTENDED = 13.inches

  object ANGLES {
    val INTAKE_ANGLE = (-40).degrees
    val STOW_ANGLE = 117.5.degrees

    val NOT_CLIPPING_ELEVATOR_THRESHOLD = 100.degrees
  }

  object PID {
    // PID Constants
    val SIM_PIVOT_KP: ProportionalGain<Radian, Volt> = 1.0.volts / 1.0.degrees
    val SIM_PIVOT_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val SIM_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.0.volts / 1.0.degrees.perSecond

    val REAL_PIVOT_KP: ProportionalGain<Radian, Volt> = 3.0.volts / 1.0.degrees
    val REAL_PIVOT_KI: IntegralGain<Radian, Volt> = 0.0.volts / (1.0.degrees * 1.0.seconds)
    val REAL_PIVOT_KD: DerivativeGain<Radian, Volt> = 0.3.volts / 1.0.degrees.perSecond
  }

  object Rollers {
    val GEAR_RATIO = 12.0 / 20.0
    val INERTIA = 5.141e-13.pounds * 1.0.inches.squared

    val IDLE_VOLTAGE = 0.volts

    val VOLTAGE_COMPENSATION = 12.volts

    val STATOR_CURRENT_LIMIT = 40.amps
    val SUPPLY_CURRENT_LIMIT = 40.amps

    val INTAKE_VOLTAGE = 6.5.volts
    val EJECT_VOLTAGE = -(1.5).volts
  }
}

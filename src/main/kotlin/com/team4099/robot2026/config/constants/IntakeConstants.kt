package com.team4099.robot2026.config.constants

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.Robot
import edu.wpi.first.wpilibj.DriverStation
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.grams
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
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

  val STATOR_CURRENT_LIMIT = 80.amps
  val SUPPLY_CURRENT_LIMIT = 40.amps

  val VOLTAGE_COMPENSATION = 12.0.volts

  val MAX_VELOCITY = 2000.rotations.perSecond
  val MAX_ACCELERATION = 2000.rotations.perSecond.perSecond
  val MAX_JERK = 6000.rotations.perSecond.perSecond.perSecond

  val SIM_VELOCITY = 400.degrees.perSecond
  val SIM_ACCELERATION = 400.degrees.perSecond.perSecond

  val LENGTH_EXTENDED = 0.0.inches

  val FORCE_HOME_INTAKE_VOLTAGE = -4.volts

  object ANGLES {
    val INTAKE_ANGLE = PIVOT_MIN_ANGLE
    val STOW_ANGLE = PIVOT_MAX_ANGLE
    val IDLE_ANGLE: Angle
      get() =
          if (DriverStation.isAutonomous() && Clock.timestamp - Robot.autoStartTime < 0.3.seconds)
              STOW_ANGLE
          else 0.degrees

    val EJECT_ANGLE = INTAKE_ANGLE

    val CLIMB_ANGLE = PIVOT_MAX_ANGLE

    val FORCE_UP_ANGLE = 110.degrees
    val FORCE_HALFUP_ANGLE = 50.degrees
    val FORCE_HALFDOWN_ANGLE = 20.degrees
    val FORCE_DOWN_ANGLE = INTAKE_ANGLE - 2.degrees
  }

  object PID {
    // PID Constants
    val REAL_PIVOT_KP: ProportionalGain<Radian, Volt> = 35.volts / 1.0.radians
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

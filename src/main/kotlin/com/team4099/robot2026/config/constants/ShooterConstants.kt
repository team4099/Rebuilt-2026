package com.team4099.robot2026.config.constants

import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Ampere
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
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.meterSquared
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.kilo
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object ShooterConstants {
  val SHOOTER_HEIGHT = 19.3.inches
  val SHOOTER_ANGLE = 75.degrees
  val SHOOTER_OFFSET = Transform2d(Translation2d(7.7.inches, -(3.6).inches), 0.radians)

  val GEAR_RATIO: Double = 24.0 / 24.0
  val SUPPLY_CURRENT_LIMIT = 40.0.amps
  val STATOR_CURRENT_LIMIT = 80.0.amps
  val VOLTAGE_COMPENSATION = 12.0.volts
  val MOMENT_OF_INERTIA = 0.0035914.kilo.grams.meterSquared

  val MAX_ACCELERATION: AngularAcceleration = 1000.rotations.perSecond.perSecond

  val SHOOTER_TOLERANCE = 75.rotations.perMinute

  object VELOCITIES {
    val MINIMUM_LAUNCH_VELOCITY = 25.rotations.perSecond
    val MAXIMUM_LAUNCH_VELOCITY = 85.rotations.perSecond
    val IDLE_VELOCITY =
        if (Constants.Tuning.TUNING_MODE) 0.rotations.perSecond else 0.rotations.perSecond
    val MANUAL_SHOOTING = 1800.rotations.perMinute
  }

  object PID {
    val REAL_KP0: ProportionalGain<Velocity<Radian>, Ampere> = 8.amps / 1.0.radians.perSecond
    val REAL_KI0: IntegralGain<Velocity<Radian>, Ampere> =
        0.0.amps / (1.0.radians.perSecond * 1.0.seconds)
    val REAL_KD0: DerivativeGain<Velocity<Radian>, Ampere> =
        0.0.amps / (1.0.radians.perSecond / 1.0.seconds)

    val REAL_KP1: ProportionalGain<Velocity<Radian>, Ampere> = 11.amps / 1.0.radians.perSecond
    val REAL_KI1: IntegralGain<Velocity<Radian>, Ampere> =
        0.0.amps / (1.0.radians.perSecond * 1.0.seconds)
    val REAL_KD1: DerivativeGain<Velocity<Radian>, Ampere> =
        0.0.amps / (1.0.radians.perSecond / 1.0.seconds)

    val REAL_KS0: StaticFeedforward<Ampere> = 10.75.amps
    val REAL_KV0: VelocityFeedforward<Radian, Ampere> = 0.088.amps / 1.radians.perSecond
    val REAL_KA0: AccelerationFeedforward<Radian, Ampere> =
        (MOMENT_OF_INERTIA.inKilogramsMeterSquared /
                (Constants.MOTOR_CONSTANTS.KRAKENX60FOC_kT / GEAR_RATIO))
            .amps / 1.radians.perSecond.perSecond
    val REAL_KS1: StaticFeedforward<Ampere> = 10.75.amps
    val REAL_KV1: VelocityFeedforward<Radian, Ampere> = 0.088.amps / 1.radians.perSecond
    val REAL_KA1: AccelerationFeedforward<Radian, Ampere> =
        (MOMENT_OF_INERTIA.inKilogramsMeterSquared /
                (Constants.MOTOR_CONSTANTS.KRAKENX60FOC_kT / GEAR_RATIO))
            .amps / 1.radians.perSecond.perSecond

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

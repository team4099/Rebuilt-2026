package com.team4099.robot2025.config.constants

import edu.wpi.first.wpilibj.RobotBase
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.feet
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.perMeterPerSecond
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.radiansPerSecondPerRadiansPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.sqrt

object DrivetrainConstants {
  const val TELEOP_TURNING_SPEED_PERCENT = 0.6

  val WHEEL_DIAMETER = (2 * 2).inches
  val DRIVETRAIN_LENGTH = 28.5.inches
  val DRIVETRAIN_WIDTH = 28.5.inches

  val BUMPER_WIDTH = 3.25.inches

  var DRIVE_SETPOINT_MAX = 16.feet.perSecond
  val TURN_SETPOINT_MAX =
    (DRIVE_SETPOINT_MAX.inMetersPerSecond / DRIVETRAIN_LENGTH.inMeters / 2 * sqrt(2.0))
      .radians
      .perSecond // 648

  // cruise velocity and accel for steering motor
  val STEERING_VEL_MAX = 1000.degrees.perSecond
  val STEERING_ACCEL_MAX = 1000.degrees.perSecond.perSecond

  val MAX_AUTO_VEL = 3.meters.perSecond // 4
  val MAX_AUTO_ACCEL = 4.meters.perSecond.perSecond // 3

  val OBJECT_APPROACH_SPEED = 2.meters.perSecond

  const val MK4_DRIVE_SENSOR_GEAR_RATIO = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)
  const val MK4I_STEERING_SENSOR_GEAR_RATIO = 7.0 / 150.0
  const val MK4N_STEERING_SENSOR_GEAR_RATIO = 1.0 / 18.75

  val STEERING_SUPPLY_CURRENT_LIMIT = 20.0.amps
  val DRIVE_SUPPLY_CURRENT_LIMIT = 50.0.amps

  val STEERING_STATOR_CURRENT_LIMIT = 20.0.amps
  val DRIVE_STATOR_CURRENT_LIMIT = 40.0.amps

  val STEERING_COMPENSATION_VOLTAGE = 10.volts
  val DRIVE_COMPENSATION_VOLTAGE = 12.volts

  const val NITRILE_WHEEL_COF = 1.2

  val INITIAL_SIM_POSE = Pose2d(3.meters, 3.meters, 0.radians).pose2d

  object PID {
    val AUTO_POS_KP: ProportionalGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return 3.15.meters.perSecond / 1.0.meters // todo:4
        } else {
          return 30.0.meters.perSecond / 1.0.meters
        }
      }
    val AUTO_POS_KI: IntegralGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return 0.0.meters.perSecond / (1.0.meters * 1.0.seconds)
        } else {
          return 0.0.meters.perSecond / (1.0.meters * 1.0.seconds)
        }
      }

    val AUTO_POS_KD: DerivativeGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return (0.6.meters.perSecond / (1.0.meters.perSecond))
            .metersPerSecondPerMetersPerSecond // todo: 0.25
        } else {
          return (0.5.meters.perSecond / (1.0.meters.perSecond)).metersPerSecondPerMetersPerSecond
        }
      }

    val LIMELIGHT_THETA_KP = 4.0.degrees.perSecond / 1.degrees
    val LIMELIGHT_THETA_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val LIMELIGHT_THETA_KD =
      (0.1.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val AUTO_THETA_PID_KP = (30.degrees.perSecond / 1.degrees)
    val AUTO_THETA_PID_KI = (0.0.radians.perSecond / (1.radians * 1.seconds))
    val AUTO_THETA_PID_KD =
      (2.7.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val AUTO_REEF_PID_KP = (2.9.radians.perSecond / 1.radians)
    val AUTO_REEF_PID_KI = (0.0.radians.perSecond / (1.radians * 1.seconds))
    val AUTO_REEF_PID_KD =
      (0.4.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val TELEOP_THETA_PID_KP = 5.5.degrees.perSecond / 1.degrees
    val TELEOP_THETA_PID_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val TELEOP_THETA_PID_KD =
      (0.3.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val TELEOP_X_PID_KP = 2.8.meters.perSecond / 1.meters
    val TELEOP_X_PID_KI = 0.0.meters.perSecond / (1.meters * 1.seconds)
    val TELEOP_X_PID_KD = 0.01.meters.perSecond.perMeterPerSecond

    val TELEOP_Y_PID_KP = 2.6.meters.perSecond / 1.meters
    val TELEOP_Y_PID_KI = 0.0.meters.perSecond / (1.meters * 1.seconds)
    val TELEOP_Y_PID_KD = 0.0.meters.perSecond.perMeterPerSecond

    val OBJECT_ALIGN_KP = 15.degrees.perSecond / 1.degrees
    val OBJECT_ALIGN_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val OBJECT_ALIGN_KD =
      (2.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val SIM_TELEOP_Y_PID_KP = TELEOP_Y_PID_KP
    val SIM_TELEOP_Y_PID_KI = TELEOP_Y_PID_KI
    val SIM_TELEOP_Y_PID_KD = TELEOP_Y_PID_KD

    val SIM_TELEOP_X_PID_KP = TELEOP_X_PID_KP
    val SIM_TELEOP_X_PID_KI = TELEOP_X_PID_KI
    val SIM_TELEOP_X_PID_KD = TELEOP_X_PID_KD

    val SIM_AUTO_THETA_PID_KP = AUTO_REEF_PID_KP
    val SIM_AUTO_THETA_PID_KI = AUTO_REEF_PID_KI
    val SIM_AUTO_THETA_PID_KD = AUTO_REEF_PID_KD

    val STEERING_KP = 10.0.volts / 45.degrees
    val STEERING_KI = 0.0.volts.perDegreeSeconds
    val STEERING_KD = 0.0.volts.perDegreePerSecond
    val STEERING_KV = 0.0.volts / 1.0.radians.perSecond

    val DRIVE_KP = 1.52.volts / 1.meters.perSecond
    val DRIVE_KI = 0.0.volts / (1.meters.perSecond * 1.seconds)
    val DRIVE_KD = 0.1.volts / 1.meters.perSecond.perSecond

    val DRIVE_KS = 0.236.volts
    val DRIVE_KV = 2.117.volts / 1.0.meters.perSecond
    val DRIVE_KA = 0.0.volts / 1.0.meters.perSecond.perSecond

    val SIM_DRIVE_KP = DRIVE_KP
    val SIM_DRIVE_KI = DRIVE_KI
    val SIM_DRIVE_KD = DRIVE_KD
    val SIM_DRIVE_KS = DRIVE_KS
    val SIM_DRIVE_KV = DRIVE_KV
    val SIM_DRIVE_KA = DRIVE_KA

    val SIM_STEERING_KP = STEERING_KP
    val SIM_STEERING_KI = STEERING_KI
    val SIM_STEERING_KD = STEERING_KD
    val SIM_STEERING_KV = STEERING_KV
  }
}

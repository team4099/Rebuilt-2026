package com.team4099.robot2026.config.constants

import com.team4099.robot2026.subsystems.drivetrain.generated.AlphaBotTunerConstants
import com.team4099.robot2026.subsystems.drivetrain.generated.CompBotTunerConstants
import com.team4099.robot2026.subsystems.drivetrain.generated.TestBotTunerConstants
import edu.wpi.first.wpilibj.RobotBase
import java.util.function.Supplier
import kotlin.math.sqrt
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
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

object DrivetrainConstants {
  val tunerConstants
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.COMPBOT -> CompBotTunerConstants
          Constants.WHOAMI.ALPHABOT -> AlphaBotTunerConstants
          Constants.WHOAMI.TESTBOT -> TestBotTunerConstants
        }

  const val TELEOP_TURNING_SPEED_PERCENT = 0.6

  val WHEEL_RADIUS: Length
    get() =
        when (Constants.Universal.whoami) {
          Constants.WHOAMI.COMPBOT,
          Constants.WHOAMI.ALPHABOT -> 2.039.inches
          Constants.WHOAMI.TESTBOT -> 1.96.inches
        }

  val DRIVETRAIN_LENGTH: Length
    get() =
        when (Constants.Universal.whoami) {
          else -> 28.5.inches
        }

  val DRIVETRAIN_WIDTH: Length
    get() =
        when (Constants.Universal.whoami) {
          else -> 28.5.inches
        }

  val BUMPER_WIDTH: Length
    get() =
        when (Constants.Universal.whoami) {
          else -> 3.25.inches
        }

  val DRIVE_SETPOINT_MAX
    get() = tunerConstants.kSpeedAt12Volts

  val TURN_SETPOINT_MAX
    get() =
        (DRIVE_SETPOINT_MAX.inMetersPerSecond / DRIVETRAIN_LENGTH.inMeters / 2 * sqrt(2.0))
            .radians
            .perSecond // 648

  // cruise velocity and accel for steering motor
  val STEERING_VEL_MAX = 1000.degrees.perSecond
  val STEERING_ACCEL_MAX = 1000.degrees.perSecond.perSecond

  val MAX_AUTO_VEL = 3.meters.perSecond // 4
  val MAX_AUTO_ACCEL = 4.meters.perSecond.perSecond // 3

  val OBJECT_APPROACH_SPEED = 2.meters.perSecond

  val STEERING_SUPPLY_CURRENT_LIMIT = 20.0.amps
  val DRIVE_SUPPLY_CURRENT_LIMIT = 50.0.amps

  val STEERING_STATOR_CURRENT_LIMIT = 20.0.amps
  val DRIVE_STATOR_CURRENT_LIMIT = 50.0.amps

  val STEERING_COMPENSATION_VOLTAGE = 10.volts
  val DRIVE_COMPENSATION_VOLTAGE = 12.volts

  private const val NITRILE_WHEEL_COF = 1.2
  private const val MOLDED_TPU_WHEEL_COF = 1.4

  val CURRENT_COF =
      when (Constants.Universal.whoami) {
        Constants.WHOAMI.COMPBOT,
        Constants.WHOAMI.ALPHABOT -> MOLDED_TPU_WHEEL_COF
        Constants.WHOAMI.TESTBOT -> NITRILE_WHEEL_COF
      }

  val INITIAL_SIM_POSE = Pose3d(3.meters, 3.meters, 0.meters, Rotation3d()).pose3d

  object PID {
    val AUTO_POS_KP: ProportionalGain<Meter, Velocity<Meter>>
      get() {
        if (RobotBase.isReal()) {
          return 2.8.meters.perSecond / 1.0.meters // todo:3.15
        } else {
            return 25.meters.perSecond / 1.0.meters
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
          return (0.6.meters.perSecond / (1.0.meters.perSecond)) // 0.6
              .metersPerSecondPerMetersPerSecond // todo: 0.25
        } else {
          return (0.5.meters.perSecond / (1.0.meters.perSecond)).metersPerSecondPerMetersPerSecond
        }
      }

    val LIMELIGHT_THETA_KP = 4.0.degrees.perSecond / 1.degrees
    val LIMELIGHT_THETA_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val LIMELIGHT_THETA_KD =
        (0.1.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

      val AUTO_THETA_PID_KP = (2.5.degrees.perSecond / 1.degrees)
    val AUTO_THETA_PID_KI = (0.0.radians.perSecond / (1.radians * 1.seconds))
      val AUTO_THETA_PID_KD =
          (0.3.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val SIM_HUB_PID_KP = (6.7.radians.perSecond / 1.radians)
    val SIM_HUB_PID_KI = (0.0.radians.perSecond / (1.radians * 1.seconds))
    val SIM_HUB_PID_KD =
        (.1.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val AUTO_REEF_PID_KP = (2.9.radians.perSecond / 1.radians)
    val AUTO_REEF_PID_KI = (0.0.radians.perSecond / (1.radians * 1.seconds))
    val AUTO_REEF_PID_KD =
        (0.4.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

    val TELEOP_THETA_PID_KP = 8.degrees.perSecond / 1.degrees
    val TELEOP_THETA_PID_KI = 0.0.degrees.perSecond / (1.degrees * 1.seconds)
    val TELEOP_THETA_PID_KD =
        (0.1.degrees.perSecond / (1.degrees / 1.seconds)).radiansPerSecondPerRadiansPerSecond

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

    val STEERING_KP = 35.volts / 1.radians
    val STEERING_KI = 0.0.volts.perDegreeSeconds
    val STEERING_KD = 0.0.volts.perDegreePerSecond
    val STEERING_KV: VelocityFeedforward<Radian, Volt>
      get() =
          when (Constants.Universal.whoami) {
            else -> 0.0.volts / 1.0.radians.perSecond
          }

    val DRIVE_KP = .15.volts / (1.meters.perSecond)
    val DRIVE_KI = 0.0.volts / (1.meters.perSecond * 1.seconds)
    val DRIVE_KD = 0.075.volts / (1.meters.perSecond.perSecond)

    val DRIVE_KS
      get() =
          when (Constants.Universal.whoami) {
            Constants.WHOAMI.COMPBOT -> 0.287.volts
            Constants.WHOAMI.ALPHABOT -> 0.24069.volts
            Constants.WHOAMI.TESTBOT -> .141.volts
          }

    val DRIVE_KV: VelocityFeedforward<Meter, Volt>
      get() =
          when (Constants.Universal.whoami) {
            Constants.WHOAMI.COMPBOT -> 0.784.volts / 1.0.meters.perSecond
            Constants.WHOAMI.ALPHABOT -> 0.74646.volts / 1.0.meters.perSecond
            Constants.WHOAMI.TESTBOT -> 0.718.volts / 1.0.meters.perSecond
          }

    val DRIVE_KA: AccelerationFeedforward<Meter, Volt>
      get() =
          when (Constants.Universal.whoami) {
            else -> 0.0.volts / 1.0.meters.perSecond.perSecond
          }

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

  object OTF_PATHS {
    val LEFT_TO_NEUTRAL_1 =
        listOf(
            Supplier { Pose2d(3.3.meters, 7.4.meters, 180.degrees) },
            Supplier { Pose2d(4.6.meters, 7.4.meters, 180.degrees) },
        )
    val LEFT_TO_NEUTRAL_2 =
        listOf(
            Supplier { Pose2d(4.6.meters, 7.4.meters, 90.degrees) },
            Supplier { Pose2d(6.meters, 7.4.meters, 90.degrees) },
        )

    val LEFT_TO_ALLIANCE_1 =
        listOf(
            Supplier { Pose2d(6.0.meters, 7.4.meters, 180.degrees) },
            Supplier { Pose2d(4.6.meters, 7.4.meters, 180.degrees) },
        )
    val LEFT_TO_ALLIANCE_2 =
        listOf(
            Supplier { Pose2d(4.6.meters, 7.4.meters, 180.degrees) },
            Supplier { Pose2d(3.3.meters, 7.4.meters, 180.degrees) },
        )
    val RIGHT_TO_NEUTRAL =
        listOf(
            Supplier { Pose2d(3.326.meters, 0.86.meters, 0.degrees) },
            Supplier { Pose2d(4.629.meters, 0.631.meters, 0.degrees) },
            Supplier { Pose2d(6.meters, 0.86.meters, 0.degrees) },
        )
    val RIGHT_TO_ALLIANCE =
        listOf(
            Supplier { Pose2d(6.meters, 0.86.meters, 180.degrees) },
            Supplier { Pose2d(4.629.meters, 0.631.meters, 180.degrees) },
            Supplier { Pose2d(3.326.meters, 0.86.meters, 180.degrees) },
        )
    val CLIMB_BOTTOM =
        Pair(
            Supplier { Pose2d(1.195.meters, 2.5.meters, 90.degrees) },
            Supplier { Pose2d(1.195.meters, 2.95.meters, 90.degrees) })
    val CLIMB_TOP =
        Pair(
            Supplier { Pose2d(.98.meters, 5.meters, -90.degrees) },
            Supplier { Pose2d(.98.meters, 4.52.meters, -90.degrees) })
  }
}

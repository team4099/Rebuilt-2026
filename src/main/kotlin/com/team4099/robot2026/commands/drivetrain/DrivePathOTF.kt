package com.team4099.robot2026.commands.drivetrain

import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.Waypoint
import com.pathplanner.lib.util.DriveFeedforwards
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.util.AllianceFlipUtil
import edu.wpi.first.math.geometry.Pose2d as WPIPose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds as WPIChassisSpeeds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController.Companion.GoalEndState
import org.team4099.lib.pplib.PathPlannerHolonomicDriveController.Companion.PathConstraints
import org.team4099.lib.pplib.PathPlannerRotationPID
import org.team4099.lib.pplib.PathPlannerTranslationPID
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadian
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianPerSecond
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianSeconds
import org.team4099.lib.units.derived.metersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.perMeter
import org.team4099.lib.units.derived.perMeterSeconds
import org.team4099.lib.units.derived.perRadian
import org.team4099.lib.units.derived.perRadianPerSecond
import org.team4099.lib.units.derived.perRadianSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

/**
 * @property drivetrain
 * @property driveX
 * @property driveY
 * @property turn
 * @property poseReferenceSupplier Supplier of current drivetrain pose.
 * @property poses List of poses for the path to go through. **WARNING: The rotation of each pose
 *   should be the direction of travel, NOT the rotation of the swerve chassis. See
 *   [PathPlanner documentation](https://pathplanner.dev/pplib-create-a-path-on-the-fly.html)**.
 *   These values can be fetched from the heading field when creating a path in PathPlanner.
 * @property initialHeading Initial heading for the path. Does not need to be super precise.
 * @property goalEndState Goal end state for chassis.
 */
class DrivePathOTF(
    private val drivetrain: Drive,
    private val driveX: DoubleSupplier,
    private val driveY: DoubleSupplier,
    private val turn: DoubleSupplier,
    private val poseReferenceSupplier: Supplier<WPIPose2d>,
    private val poses: List<Supplier<Pose2d>>,
    private val initialHeading: Angle,
    private val goalEndState: GoalEndState
) : Command() {
  private val DRIVE_ESCAPE_THRESHOLD = 0.4
  private val TURN_ESCAPE_THRESHOLD = 0.4

  private lateinit var command: FollowPathCommand

  private val thetakP =
      LoggedTunableValue(
          "Pathfollow/thetakP",
          Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian }))
  private val thetakI =
      LoggedTunableValue(
          "Pathfollow/thetakI",
          Pair(
              { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }))
  private val thetakD =
      LoggedTunableValue(
          "Pathfollow/thetakD",
          Pair(
              { it.inRadiansPerSecondPerRadianPerSecond },
              { it.radians.perSecond.perRadianPerSecond }))

  private val poskP =
      LoggedTunableValue(
          "Pathfollow/posKP",
          DrivetrainConstants.PID.AUTO_POS_KP,
          Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter }))
  private val poskI =
      LoggedTunableValue(
          "Pathfollow/posKI",
          DrivetrainConstants.PID.AUTO_POS_KI,
          Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds }))
  private val poskD =
      LoggedTunableValue(
          "Pathfollow/posKD",
          DrivetrainConstants.PID.AUTO_POS_KD,
          Pair(
              { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }))

  private val ppHolonomicDriveController: PathPlannerHolonomicDriveController
  private val pathConstraints: PathConstraints

  init {
    addRequirements(drivetrain)

    if (RobotBase.isReal()) {
      thetakP.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.AUTO_THETA_PID_KD)
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    }

    ppHolonomicDriveController =
        PathPlannerHolonomicDriveController(
            PathPlannerTranslationPID(poskP.get(), poskI.get(), poskD.get()),
            PathPlannerRotationPID(thetakP.get(), thetakI.get(), thetakD.get()),
            Constants.Universal.LOOP_PERIOD_TIME)

    pathConstraints =
        PathConstraints(
            DrivetrainConstants.DRIVE_SETPOINT_MAX,
            DrivetrainConstants.MAX_AUTO_ACCEL,
            DrivetrainConstants.STEERING_VEL_MAX,
            DrivetrainConstants.STEERING_ACCEL_MAX)
  }

  override fun initialize() {
    ppHolonomicDriveController.reset(drivetrain.pose.toPose2d(), drivetrain.chassisSpeeds)

    val waypoints: List<Waypoint> =
        PathPlannerPath.waypointsFromPoses(
            buildList(capacity = poses.size + 1) {
              val pose = poseReferenceSupplier.get()
              add(WPIPose2d(pose.x, pose.y, Rotation2d(initialHeading.inRadians)))
              addAll(poses.map { AllianceFlipUtil.apply(it.get()).pose2d })
            })

    val path =
        PathPlannerPath(
            waypoints, pathConstraints.pplibConstraints, null, goalEndState.pplibGoalEndState)

    Logger.recordOutput("DrivePathOTF/pathTrajectory", *path.pathPoses.toTypedArray())

    command =
        FollowPathCommand(
            path,
            { AllianceFlipUtil.apply(Pose2d(poseReferenceSupplier.get())).pose2d },
            { drivetrain.chassisSpeeds.chassisSpeedsWPILIB },
            { speeds: WPIChassisSpeeds, ff: DriveFeedforwards ->
              drivetrain.runSpeeds(ChassisSpeeds(speeds), flipIfRed = false)
            },
            ppHolonomicDriveController.pplibController,
            Drive.PP_CONFIG,
            { AllianceFlipUtil.shouldFlip() },
        )

    command.initialize()
  }

  override fun execute() {
    command.execute()
  }

  override fun isFinished(): Boolean {
    val poseDelta = poses.last().get().minus(drivetrain.pose.toPose2d())
    return command.isFinished &&
        poseDelta.translation.x < 2.inches &&
        poseDelta.translation.y < 2.inches ||
        driveX.asDouble >= DRIVE_ESCAPE_THRESHOLD ||
        driveY.asDouble >= DRIVE_ESCAPE_THRESHOLD ||
        turn.asDouble >= TURN_ESCAPE_THRESHOLD
  }

  override fun end(interrupted: Boolean) {
    // the end method of FollowPathCommand is very nice. they check if not interrupted and
    // goalendstate has velocity < 0.1 mps and with that, determines whether or not the drivetrain
    // should stop moving. tl;dr: we don't have to set drivetrain.closedloop again
    command.end(interrupted)
  }

  companion object {
    fun warmupCommand() {
      FollowPathCommand.warmupCommand()
    }

    fun allianceZoneToNeutralInLeftTrench(drivetrain: Drive): DrivePathOTF {
      return DrivePathOTF(
          drivetrain,
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { drivetrain.pose.toPose2d().pose2d },
          DrivetrainConstants.OTF_PATHS.LEFT_TO_NEUTRAL,
          0.0.degrees,
          if (!AllianceFlipUtil.shouldFlip()) {
            GoalEndState(0.0.meters.perSecond, 180.degrees)
          } else {
            GoalEndState(0.0.meters.perSecond, 0.degrees)
          })
    }

    fun allianceZoneToNeutralInRightTrench(drivetrain: Drive): DrivePathOTF {
      return DrivePathOTF(
          drivetrain,
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { drivetrain.pose.toPose2d().pose2d },
          DrivetrainConstants.OTF_PATHS.RIGHT_TO_NEUTRAL,
          0.0.degrees,
          if (!AllianceFlipUtil.shouldFlip()) {
            GoalEndState(0.0.meters.perSecond, 180.degrees)
          } else {
            GoalEndState(0.0.meters.perSecond, 0.degrees)
          })
    }

    fun neutralZoneToAllianceInLeftTrench(drivetrain: Drive): DrivePathOTF {
      return DrivePathOTF(
          drivetrain,
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { drivetrain.pose.toPose2d().pose2d },
          DrivetrainConstants.OTF_PATHS.LEFT_TO_ALLIANCE,
          0.0.degrees,
          if (!AllianceFlipUtil.shouldFlip()) {
            GoalEndState(0.0.meters.perSecond, 0.degrees)
          } else {
            GoalEndState(0.0.meters.perSecond, 180.degrees)
          })
    }

    fun neutralZoneToAllianceInRightTrench(drivetrain: Drive): DrivePathOTF {
      return DrivePathOTF(
          drivetrain,
          { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
          { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
          { drivetrain.pose.toPose2d().pose2d },
          DrivetrainConstants.OTF_PATHS.RIGHT_TO_ALLIANCE,
          0.0.degrees,
          if (!AllianceFlipUtil.shouldFlip()) {
            GoalEndState(0.0.meters.perSecond, 0.degrees)
          } else {
            GoalEndState(0.0.meters.perSecond, 180.degrees)
          })
    }

    fun alignClimbBottom(drivetrain: Drive): SequentialCommandGroup {
      return DrivePathOTF(
              drivetrain,
              { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
              { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { drivetrain.pose.toPose2d().pose2d },
              listOf(DrivetrainConstants.OTF_PATHS.CLIMB_BOTTOM.first),
              drivetrain.pose.rotation.z,
              GoalEndState(0.0.meters.perSecond, 90.degrees))
          .andThen(
              DrivePathOTF(
                  drivetrain,
                  { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                  { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                  { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
                  { drivetrain.pose.toPose2d().pose2d },
                  listOf(DrivetrainConstants.OTF_PATHS.CLIMB_BOTTOM.second),
                  drivetrain.pose.rotation.z,
                  GoalEndState(0.0.meters.perSecond, 90.degrees)))
    }

    fun alignClimbTop(drivetrain: Drive): SequentialCommandGroup {
      return DrivePathOTF(
              drivetrain,
              { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
              { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
              { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
              { drivetrain.pose.toPose2d().pose2d },
              listOf(DrivetrainConstants.OTF_PATHS.CLIMB_TOP.first),
              drivetrain.pose.rotation.z,
              GoalEndState(0.0.meters.perSecond, -90.degrees))
          .andThen(
              DrivePathOTF(
                  drivetrain,
                  { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                  { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                  { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
                  { drivetrain.pose.toPose2d().pose2d },
                  listOf(DrivetrainConstants.OTF_PATHS.CLIMB_TOP.second),
                  drivetrain.pose.rotation.z,
                  GoalEndState(0.0.meters.perSecond, -90.degrees)))
    }
  }
}

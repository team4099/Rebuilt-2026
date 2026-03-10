package com.team4099.robot2026.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.RobotContainer
import com.team4099.robot2026.RobotContainer.hasAligned
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.units.LinearVelocityUnit
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.LinearVelocity as WPILinearVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.PI
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond

/**
 * Aim for the HUB or to pass, depending on current position on the field and which option is legal.
 * This command does NOT activate the shooter; that is the responsibility of the superstructure's
 * state machine.
 *
 * Note: This command assumes a flat drivetrain on z = 0.
 *
 * Note: This command never ends.
 *
 * @param drivetrain
 * @see com.team4099.robot2026.subsystems.superstructure.shooter.Shooter.calculateLaunchData
 * @author Nathan Arega, Ryan Chung
 */
class AimCommand(
    private val drivetrain: Drive,
    private val vision: Vision,
) : Command() {

  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private var timeout = -1.seconds
  private var startTime = -1.seconds

  private var startedInAuto = false

  private var convergedPose = Pose3d()

  init {
    thetaPID =
        PIDController(
            DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KD)

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset()

    convergedPose = Pose3d()
    hasAligned = false
    startTime = Clock.timestamp
    startedInAuto = DriverStation.isAutonomous()

    RobotContainer.isAligning = true
  }

  override fun execute() {
    val currentlyTrackedPose: Pose3d

    val expectedPose = drivetrain.pose + vision.lastTagVisionUpdate.robotTTargetTag

    if (convergedPose == Pose3d()) {
      val robotTExpected = drivetrain.pose.relativeTo(expectedPose)
      if (robotTExpected.x.absoluteValue < 3.inches &&
          robotTExpected.y.absoluteValue < 3.inches &&
          (robotTExpected.rotation.z < 3.degrees ||
              robotTExpected.rotation.z > (-180.degrees + 3.degrees))) {
        currentlyTrackedPose = expectedPose
        convergedPose = expectedPose
      } else {
        currentlyTrackedPose = drivetrain.pose
      }
    } else {
      currentlyTrackedPose = convergedPose
    }

    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", true)

    val (distanceToHub, launchSpeed, timeOfFlight, wantedRotation) =
        Shooter.calculateLaunchData(currentlyTrackedPose.toPose2d(), ChassisSpeeds())

    val thetaVel = thetaPID.calculate(currentlyTrackedPose.rotation.z, wantedRotation)

    CustomLogger.recordOutput("FaceHubCommand/thetaError", thetaPID.error.inDegrees)

    CustomLogger.recordOutput(
        "FaceHubCommand/wantedPose",
        Pose2d(currentlyTrackedPose.x, currentlyTrackedPose.y, wantedRotation).pose2d)

    hasAligned = thetaPID.error.absoluteValue < 3.degrees

    CustomLogger.recordOutput("FaceHubCommand/hasAligned", hasAligned)

    if (!hasAligned) {
      drivetrain.runSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0.meters.perSecond, 0.meters.perSecond, thetaVel, drivetrain.pose.rotation.z))
    } else {
      drivetrain.stopWithX()
    }

    if (RobotBase.isSimulation() &&
        (hasAligned &&
            Clock.timestamp.inSeconds % 1 < 0.04 &&
            RobotContainer.superstructure.currentState ==
                Superstructure.Companion.SuperstructureStates.SCORE ||
            DriverStation.isAutonomous())) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              RebuiltFuelOnFly(
                  drivetrain.pose.translation.toTranslation2d().translation2d,
                  ShooterConstants.SHOOTER_OFFSET.translation.translation2d,
                  edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
                      drivetrain.chassisSpeeds.chassisSpeedsWPILIB,
                      drivetrain.rotation.z.inRotation2ds),
                  (drivetrain.pose.rotation.z + ShooterConstants.SHOOTER_OFFSET.rotation)
                      .inRotation2ds,
                  Meters.of(ShooterConstants.SHOOTER_HEIGHT.inMeters),
                  WPILinearVelocity.ofBaseUnits(
                      launchSpeed.inMetersPerSecond, LinearVelocityUnit.combine(Meters, Seconds)),
                  Degrees.of(ShooterConstants.SHOOTER_ANGLE.inDegrees)))
    }
  }

  override fun isFinished(): Boolean {
    return startedInAuto xor DriverStation.isAutonomous()
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordOutput("FaceHubCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    RobotContainer.isAligning = false

    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", false)
  }
}

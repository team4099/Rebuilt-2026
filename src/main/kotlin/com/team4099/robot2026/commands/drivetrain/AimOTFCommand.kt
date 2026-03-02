package com.team4099.robot2026.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.RobotContainer
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.driver.DriverProfile
import edu.wpi.first.units.LinearVelocityUnit
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.LinearVelocity as WPILinearVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
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
 * Velocity caluclations intend for this to be used while shooting and moving "on-the-fly". This
 * command does NOT activate the shooter; that is the responsibility of the superstructure's state
 * machine.
 *
 * Note: This command assumes a flat drivetrain on z = 0.
 *
 * Note: This command never ends.
 *
 * @param drivetrain
 * @param vSup Supplier for drivetrain field-relative chassis speeds
 * @see com.team4099.robot2026.subsystems.superstructure.shooter.Shooter.calculateLaunchData
 * @author Nathan Arega, Ryan Chung
 */
class AimOTFCommand(
    private val drivetrain: Drive,
    private val vSup: Supplier<Pair<LinearVelocity, LinearVelocity>>
) : Command() {

  /**
   * Aim for the HUB or to pass, depending on current position on the field and which option is
   * legal. Velocity caluclations intend for this to be used while shooting and moving "on-the-fly".
   * This command does NOT activate the shooter; that is the responsibility of the superstructure's
   * state machine.
   *
   * This constructor is meant to be used during driver-controlled field-oriented driving.
   *
   * @param drivetrain
   * @param driveX
   * @param driveY
   * @param slowMode
   * @param driver
   */
  constructor(
      drivetrain: Drive,
      driveX: () -> Double,
      driveY: () -> Double,
      slowMode: () -> Boolean,
      driver: DriverProfile
  ) : this(drivetrain, Supplier { driver.driveSpeedClampedSupplier(driveX, driveY, slowMode) })

  /**
   * Aim for the HUB or to pass, depending on current position on the field and which option is
   * legal. Velocity caluclations intend for this to be used while shooting and moving "on-the-fly".
   * This command does NOT activate the shooter; that is the responsibility of the superstructure's
   * state machine.
   *
   * This constructor is meant to be used during autonomous, when another command is controlling the
   * translation of the robot. You must pass in a timeout.
   *
   * @param drivetrain
   * @param timeout Time for the command to run
   */
  constructor(
      drivetrain: Drive,
      timeout: Time
  ) : this(drivetrain, Supplier { Pair(0.meters.perSecond, 0.meters.perSecond) }) {
    this.timeout = timeout
  }

  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private val MAX_VELOCITY_RADIUS = 1.5.meters.perSecond
  private var timeout = -1.seconds
  private var startTime = -1.seconds

  private var startedInAuto = false

  init {
    thetaPID =
        PIDController(
            DrivetrainConstants.PID.AUTO_THETA_PID_KP,
            DrivetrainConstants.PID.AUTO_THETA_PID_KI,
            DrivetrainConstants.PID.AUTO_THETA_PID_KD)

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset()

    hasAligned = false
    startTime = Clock.timestamp
    startedInAuto = DriverStation.isAutonomous()

    RobotContainer.isAligning = true
  }

  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", true)

    val (distanceToHub, launchSpeed, timeOfFlight, wantedRotation) =
        Shooter.calculateLaunchData(drivetrain.pose.toPose2d(), drivetrain.chassisSpeeds)

    val thetaVel = thetaPID.calculate(drivetrain.rotation.z, wantedRotation)

    CustomLogger.recordOutput("FaceHubCommand/thetaError", thetaPID.error.inDegrees)

    CustomLogger.recordOutput(
        "FaceHubCommand/wantedPose",
        Pose2d(drivetrain.pose.x, drivetrain.pose.y, wantedRotation).pose2d)

    hasAligned = thetaPID.error.absoluteValue < 2.degrees

    CustomLogger.recordOutput("FaceHubCommand/hasAligned", hasAligned)

    if (DriverStation.isAutonomous()) {
      // Use planned path velocities, dont adjust
      drivetrain.runRotationWhileKeepingTranslation(thetaVel)
    } else {
      // Take the drivers speed being inputted, and clamp the magnitude
      // of the drive vector to < MAX_VELOCITY_RADIUS meters per second
      var (speedX, speedY) = vSup.get()
      val speedMagnitude =
          sqrt(speedX.inMetersPerSecond.pow(2) + speedY.inMetersPerSecond.pow(2)).meters.perSecond

      if (speedMagnitude > 0.1.meters.perSecond || !hasAligned) {
        if (speedMagnitude > MAX_VELOCITY_RADIUS) {
          // Convert to unit vector and then * MAX_VELOCITY_RADIUS
          speedX = speedX / speedMagnitude.inMetersPerSecond * MAX_VELOCITY_RADIUS.inMetersPerSecond
          speedY = speedY / speedMagnitude.inMetersPerSecond * MAX_VELOCITY_RADIUS.inMetersPerSecond
        }

        drivetrain.runSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speedX, speedY, thetaVel, drivetrain.pose.rotation.z))
      } else {
        drivetrain.stopWithX()
      }
    }

    if (RobotBase.isSimulation() &&
        hasAligned &&
        Clock.timestamp.inSeconds % 1 < 0.04 &&
        RobotContainer.superstructure.currentState ==
            Superstructure.Companion.SuperstructureStates.SCORE || DriverStation.isAutonomous()) {
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
    return timeout > 0.seconds && Clock.timestamp - startTime > timeout ||
        startedInAuto xor DriverStation.isAutonomous()
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordOutput("FaceHubCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    RobotContainer.isAligning = false

    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", false)
  }

  companion object {
    var hasAligned: Boolean = false
  }
}

package com.team4099.robot2025.commands.drivetrain

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team4099.lib.controller.PIDController
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegree
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreePerSecond
import org.team4099.lib.units.derived.inDegreesPerSecondPerDegreeSeconds
import org.team4099.lib.units.derived.perDegree
import org.team4099.lib.units.derived.perDegreePerSecond
import org.team4099.lib.units.derived.perDegreeSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI

class TargetAngleCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drive,
  val targetAngle: () -> Angle
) : Command() {

  private var thetaPID: PIDController<Radian, Velocity<Radian>>
  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetaAmpkP",
      Pair({ it.inDegreesPerSecondPerDegree }, { it.degrees.perSecond.perDegree })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetaAmpkI",
      Pair(
        { it.inDegreesPerSecondPerDegreeSeconds }, { it.degrees.perSecond.perDegreeSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inDegreesPerSecondPerDegreePerSecond },
        { it.degrees.perSecond.perDegreePerSecond }
      )
    )

  private val request =
    SwerveRequest.FieldCentric()
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

  init {
    addRequirements(drivetrain)

    thetaPID =
      PIDController(
        thetakP.get(),
        thetakI.get(),
        thetakD.get(),
      )

    if (!(RobotBase.isSimulation())) {

      thetakP.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.TELEOP_THETA_PID_KD)

      thetaPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
          DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
          DrivetrainConstants.PID.TELEOP_THETA_PID_KD
        )
    } else {
      thetakP.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP)
      thetakI.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI)
      thetakD.initDefault(DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)

      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset() // maybe do first for x?
    /*
    if (thetakP.hasChanged() || thetakI.hasChanged() || thetakD.hasChanged()) {
      thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())
    }

     */
  }

  override fun execute() {

    drivetrain.defaultCommand.end(true)
    CustomLogger.recordDebugOutput("ActiveCommands/TargetAngleCommand", true)
    Logger.recordOutput("Testing/CurrentDrivetrainRotation", drivetrain.pose.rotation.z.inDegrees)

    val thetaFeedback = thetaPID.calculate(drivetrain.pose.rotation.z, targetAngle())
    CustomLogger.recordDebugOutput("Testing/error", thetaPID.error.inDegrees)
    CustomLogger.recordDebugOutput("Testing/thetaFeedback", thetaFeedback.inDegreesPerSecond)

    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)

    drivetrain.runSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        speed.first, speed.second, thetaFeedback, drivetrain.rotation.z
      )
    )
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordDebugOutput("ActiveCommands/TargetAngleCommand", false)
    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
    val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)
    drivetrain.runSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        speed.first, speed.second, rotation, drivetrain.pose.rotation.z
      )
    )
  }
}

package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.lib.math.asPose2d
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI

class CoolerTargetTagCommand(
  private val drivetrain: Drive,
  private val vision: Vision,
  private val xTargetOffset: Length =
    DrivetrainConstants.DRIVETRAIN_LENGTH / 2 + DrivetrainConstants.BUMPER_WIDTH,
  private val yTargetOffset: Length = 0.0.inches,
  private val thetaTargetOffset: Angle = 0.0.radians,
) : Command() {

  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var yPID: PIDController<Meter, Velocity<Meter>> =
    PIDController(
      DrivetrainConstants.PID.TELEOP_Y_PID_KP,
      DrivetrainConstants.PID.TELEOP_Y_PID_KI,
      DrivetrainConstants.PID.TELEOP_Y_PID_KD
    )
  private var xPID: PIDController<Meter, Velocity<Meter>> =
    PIDController(
      DrivetrainConstants.PID.TELEOP_Y_PID_KP,
      DrivetrainConstants.PID.TELEOP_Y_PID_KI,
      DrivetrainConstants.PID.TELEOP_Y_PID_KD
    )

  private var hasThetaAligned: Boolean = false
  private var hasPointedAt: Boolean = false

  private var startTime: Time = 0.0.seconds

  init {
    addRequirements(drivetrain, vision)

    if (RobotBase.isSimulation()) {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )

      yPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_Y_PID_KD
        )

      xPID =
        PIDController(
          DrivetrainConstants.PID.SIM_TELEOP_X_PID_KP,
          DrivetrainConstants.PID.SIM_TELEOP_X_PID_KI,
          DrivetrainConstants.PID.SIM_TELEOP_X_PID_KD
        )
    } else {
      if (DriverStation.isAutonomous()) {
        thetaPID =
          PIDController(
            DrivetrainConstants.PID.AUTO_REEF_PID_KP,
            DrivetrainConstants.PID.AUTO_REEF_PID_KI,
            DrivetrainConstants.PID.AUTO_REEF_PID_KD
          )
      } else {
        thetaPID =
          PIDController(
            DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KD
          )
      }

      yPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_Y_PID_KP,
          DrivetrainConstants.PID.TELEOP_Y_PID_KI,
          DrivetrainConstants.PID.TELEOP_Y_PID_KD
        )

      xPID =
        PIDController(
          DrivetrainConstants.PID.TELEOP_X_PID_KP,
          DrivetrainConstants.PID.TELEOP_X_PID_KI,
          DrivetrainConstants.PID.TELEOP_X_PID_KD
        )
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    startTime = Clock.fpgaTime

    xPID.reset()
    yPID.reset()
    thetaPID.reset()

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    vision.isAligned = false
    vision.isAutoAligning = true
    hasThetaAligned = false
    hasPointedAt = false

    CustomLogger.recordOutput("CoolerTargetTagCommand/lastInitialized", Clock.fpgaTime.inSeconds)
  }

  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/CoolerTargetTagCommand", true)

    val lastUpdate = vision.lastTrigVisionUpdate
    val odomTTag = lastUpdate.robotTReefTag

    val exists = odomTTag != Transform2d(Translation2d(), 0.degrees)
    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTagExists", exists)
    if (!exists || Clock.fpgaTime - lastUpdate.timestamp > .5.seconds)
      end(interrupted = true) // todo kalman?

    val setpointTranslation = odomTTag.translation
    val setpointRotation = odomTTag.rotation

    CustomLogger.recordOutput("CoolerTargetTagCommand/odomTTag", odomTTag.asPose2d().pose2d)
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/expectedTagPose", drivetrain.pose.transformBy(odomTTag).pose2d
    )
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/setpointTranslation", setpointTranslation.translation2d
    )
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/currentRotation", drivetrain.rotation.inDegrees
    )
    CustomLogger.recordOutput(
      "CoolerTargetTagCommand/setpointRotation", (setpointRotation + thetaTargetOffset).inDegrees
    )

    // todo check signs and whatnot
    var xvel =
      xPID.calculate(setpointTranslation.x, xTargetOffset * setpointTranslation.x.sign) *
        setpointTranslation.x.sign
    var yvel = -yPID.calculate(setpointTranslation.y, yTargetOffset)
    var thetavel =
      thetaPID.calculate(drivetrain.rotation, setpointRotation + thetaTargetOffset) *
        if (RobotBase.isReal()) -1.0 else 1.0

    CustomLogger.recordOutput("CoolerTargetTagCommand/xvelmps", xvel.inMetersPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/yvelmps", yvel.inMetersPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("CoolerTargetTagCommand/xerror", xPID.error.inInches)
    CustomLogger.recordOutput("CoolerTargetTagCommand/yerror", yPID.error.inInches)
    CustomLogger.recordOutput("CoolerTargetTagCommand/thetaerror", thetaPID.error.inDegrees)

    CustomLogger.recordOutput("CoolerTargetTagCommand/hasThetaAligned", hasThetaAligned)
    CustomLogger.recordOutput("CoolerTargetTagCommand/hasPointedAt", hasPointedAt)

    if (hasThetaAligned || thetaPID.error.absoluteValue < 4.49.degrees) {
      hasThetaAligned = true

      drivetrain.runSpeeds(ChassisSpeeds(xvel, yvel, thetavel), flipIfRed = false)
    } else {
      drivetrain.runSpeeds(
        ChassisSpeeds(0.meters.perSecond, 0.meters.perSecond, thetavel), flipIfRed = false
      )
    }
  }

  override fun isFinished(): Boolean {
    return xPID.error < .8.inches && yPID.error < .8.inches && thetaPID.error < 2.5.degrees
  }

  override fun end(interrupted: Boolean) {
    if (!interrupted) vision.isAligned = true
    vision.isAutoAligning = false

    CustomLogger.recordOutput("CoolerTargetTagCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }

  companion object {
    fun alignLeftCommand(drivetrain: Drive, vision: Vision): CoolerTargetTagCommand {
      return CoolerTargetTagCommand(
        drivetrain,
        vision,
        xTargetOffset =
        DrivetrainConstants.DRIVETRAIN_LENGTH / 2 +
          DrivetrainConstants.BUMPER_WIDTH +
          2.0.inches,
        yTargetOffset = (12.94 / 2).inches
      )
    }

    fun alignRightCommand(drivetrain: Drive, vision: Vision): CoolerTargetTagCommand {
      return CoolerTargetTagCommand(
        drivetrain,
        vision,
        xTargetOffset =
        DrivetrainConstants.DRIVETRAIN_LENGTH / 2 +
          DrivetrainConstants.BUMPER_WIDTH +
          2.0.inches,
        yTargetOffset = (-12.94 / 2).inches
      )
    }

    fun alignCenter(drivetrain: Drive, vision: Vision): CoolerTargetTagCommand {
      return CoolerTargetTagCommand(
        drivetrain,
        vision,
        xTargetOffset =
        DrivetrainConstants.DRIVETRAIN_LENGTH / 2 +
          DrivetrainConstants.BUMPER_WIDTH +
          2.0.inches
      )
    }
  }
}

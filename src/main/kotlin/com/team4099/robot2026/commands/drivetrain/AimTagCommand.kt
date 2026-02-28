package com.team4099.robot2026.commands.drivetrain

import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import kotlin.math.PI
import kotlin.math.atan2
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

class AimTagCommand(val drivetrain: Drive, val vision: Vision) : Command() {
  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  init {
    addRequirements(drivetrain)

    if (RobotBase.isSimulation()) {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    } else {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
              DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
              DrivetrainConstants.PID.TELEOP_THETA_PID_KD)
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset()
  }

  override fun execute() {
    val lastUpdate = vision.lastTrigVisionUpdate
    if (lastUpdate.timestamp <= 0.seconds ||
        lastUpdate.robotTTargetTag.transform3d == Transform3d())
        return

    CustomLogger.recordOutput(
        "AimTagCommand/robotTTargetTag", lastUpdate.robotTTargetTag.transform3d)

    val targetAngle =
        atan2(
                lastUpdate.robotTTargetTag.translation.y.inMeters,
                lastUpdate.robotTTargetTag.translation.x.inMeters)
            .radians

    CustomLogger.recordOutput("AimTagCommand/targetAngleDegrees", targetAngle.inDegrees)

    val omega = -thetaPID.calculate(targetAngle, 0.radians)

    CustomLogger.recordOutput("AimTagCommand/omegaDPS", omega.inDegreesPerSecond)

    drivetrain.runSpeeds(ChassisSpeeds(0.meters.perSecond, 0.meters.perSecond, omega))
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    drivetrain.runSpeeds(ChassisSpeeds())
  }
}

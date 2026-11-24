package com.team4099.robot2026.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.ClusterScore
import com.team4099.robot2026.util.CustomLogger
import kotlin.math.PI
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisVelocities as ChassisSpeeds
import org.team4099.lib.units.Value
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond
import org.wpilib.command2.Command
import org.wpilib.framework.RobotBase

class TargetObjectCommand(
    private val drivetrain: Drive,
    private val vision: Vision,
    private val targetObjectClass: VisionConstants.OBJECT_CLASS
) : Command() {
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var hasThetaAligned: Boolean = false

  private var startTime: Time = 0.0.seconds

  init {
    addRequirements(drivetrain, vision)

    if (RobotBase.isSimulation()) {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
              DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD)
    } else if (org.wpilib.driverstation.RobotState.isAutonomous()) {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.AUTO_REEF_PID_KP,
              DrivetrainConstants.PID.AUTO_REEF_PID_KI,
              DrivetrainConstants.PID.AUTO_REEF_PID_KD)
    } else {
      thetaPID =
          PIDController(
              DrivetrainConstants.PID.OBJECT_ALIGN_KP,
              DrivetrainConstants.PID.OBJECT_ALIGN_KI,
              DrivetrainConstants.PID.OBJECT_ALIGN_KD)
    }
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    startTime = Clock.timestamp
    thetaPID.reset()
    hasThetaAligned = false

    CustomLogger.recordOutput("TargetObjectCommand/lastInitialized", Clock.timestamp.inSeconds)
  }

  override fun execute() {
    var robotTObject: Translation2d
    val lastUpdate = vision.lastObjectVisionUpdate[targetObjectClass.id]
    if (RobotBase.isSimulation()) {
      var fuelTranslations = vision.objectsDetected[0]
      if (fuelTranslations.isEmpty()) return
      val target =
          ClusterScore.calculateClusterScores(
              drivetrain.pose.pose2d, fuelTranslations.map { it.translation3d })
      CustomLogger.recordOutput("TargetObjectCommand/Target", target)
      robotTObject = Transform2d(drivetrain.pose, Pose2d(target)).translation
    } else {
      robotTObject = lastUpdate.robotTObject.toTranslation2d()
    }

    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", true)

    val exists = (robotTObject != Translation2d())

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectExists", exists)
    if (!exists || Clock.timestamp - lastUpdate.timestamp > .2.seconds) end(interrupted = true)

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectx", robotTObject.x.inMeters)
    CustomLogger.recordOutput("TargetObjectCommand/odomTObjecty", robotTObject.y.inMeters)

    val setpointRotation: Value<Radian> =
        robotTObject.translation2d.angle
            .orElse(org.wpilib.math.geometry.Rotation2d.ZERO)
            .radians
            .radians + drivetrain.pose.rotation

    CustomLogger.recordOutput("TargetObjectCommand/setPointRotation", setpointRotation.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/driverot", drivetrain.rotation.inDegrees)

    val thetavel =
        thetaPID.calculate(drivetrain.pose.rotation, setpointRotation) *
            if (RobotBase.isReal()) -1.0 else 1.0

    CustomLogger.recordOutput("TargetObjectCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("TargetObjectCommand/thetaerror", thetaPID.error.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/hasThetaAligned", hasThetaAligned)

    if (hasThetaAligned || thetaPID.error.absoluteValue < 8.36.degrees) {
      hasThetaAligned = true

      drivetrain.runSpeeds(
          ChassisSpeeds(DrivetrainConstants.OBJECT_APPROACH_SPEED, 0.meters.perSecond, thetavel),
          flipIfRed = false)
    } else {
      drivetrain.runSpeeds(
          ChassisSpeeds(0.meters.perSecond, 0.meters.perSecond, thetavel), flipIfRed = false)
    }
  }

  override fun end(interrupted: Boolean) {
    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", false)

    CustomLogger.recordOutput("TargetObjectCommand/interrupted", interrupted)
  }

  override fun isFinished(): Boolean {
    return false
  }
}

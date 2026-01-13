package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.config.constants.VisionConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.superstructure.Request
import com.team4099.robot2025.subsystems.superstructure.Superstructure
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
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
import kotlin.math.PI

class TargetObjectCommand(
  private val drivetrain: Drive,
  private val vision: Vision,
  private val targetObjectClass: VisionConstants.OBJECT_CLASS,
  private val superstructure: Superstructure
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
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )
    } else if (DriverStation.isAutonomous()) {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.AUTO_REEF_PID_KP,
          DrivetrainConstants.PID.AUTO_REEF_PID_KI,
          DrivetrainConstants.PID.AUTO_REEF_PID_KD
        )
    } else {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.OBJECT_ALIGN_KP,
          DrivetrainConstants.PID.OBJECT_ALIGN_KI,
          DrivetrainConstants.PID.OBJECT_ALIGN_KD
        )
    }
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }
  override fun initialize() {
    startTime = Clock.fpgaTime
    thetaPID.reset()
    hasThetaAligned = false

    CustomLogger.recordOutput("TargetObjectCommand/lastInitialized", Clock.fpgaTime.inSeconds)
  }
  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", true)

    val lastUpdate = vision.lastObjectVisionUpdate[targetObjectClass.id]
    val robotTObject = lastUpdate.robotTObject

    val exists = (robotTObject != Translation2d())

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectExists", exists)
    if (!exists || Clock.fpgaTime - lastUpdate.timestamp > .2.seconds) end(interrupted = true)

    superstructure.currentRequest = Request.SuperstructureRequest.IntakeCoral()

    CustomLogger.recordOutput("TargetObjectCommand/odomTObjectx", robotTObject.x.inMeters)
    CustomLogger.recordOutput("TargetObjectCommand/odomTObjecty", robotTObject.y.inMeters)

    val setpointRotation: Value<Radian> =
      robotTObject.translation2d.angle.radians.radians + drivetrain.pose.rotation

    CustomLogger.recordOutput("TargetObjectCommand/setPointRotation", setpointRotation.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/driverot", drivetrain.rotation.inDegrees)

    val thetavel =
      thetaPID.calculate(drivetrain.pose.rotation, setpointRotation) *
        if (RobotBase.isReal()) -1.0 else 1.0

    CustomLogger.recordOutput("TargetObjectCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("TargetObjectCommand/thetaerror", thetaPID.error.inDegrees)
    CustomLogger.recordOutput("TargetObjectCommand/hasThetaAligned", hasThetaAligned)

    if ((hasThetaAligned || thetaPID.error.absoluteValue < 8.36.degrees) &&
      superstructure.currentState ==
      Superstructure.Companion.SuperstructureStates.GROUND_INTAKE_CORAL
    ) {
      hasThetaAligned = true

      drivetrain.runSpeeds(
        ChassisSpeeds(DrivetrainConstants.OBJECT_APPROACH_SPEED, 0.meters.perSecond, thetavel),
        flipIfRed = false
      )
    } else {
      drivetrain.runSpeeds(
        ChassisSpeeds(0.meters.perSecond, 0.meters.perSecond, thetavel), flipIfRed = false
      )
    }
  }
  override fun end(interrupted: Boolean) {
    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetObjectCommand", false)

    CustomLogger.recordOutput("TargetObjectCommand/interrupted", interrupted)
  }

  override fun isFinished(): Boolean {
    return if (targetObjectClass == VisionConstants.OBJECT_CLASS.CORAL) {
      superstructure.theoreticalGamePieceHardstop != Constants.Universal.GamePiece.NONE
    } else {
      superstructure.theoreticalGamePieceArm != Constants.Universal.GamePiece.NONE
    }
  }
}

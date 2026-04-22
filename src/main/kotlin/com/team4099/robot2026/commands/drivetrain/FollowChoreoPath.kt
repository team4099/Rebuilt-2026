package com.team4099.robot2026.commands.drivetrain

import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.team4099.lib.hal.Clock
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.util.AllianceFlipUtil
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.Velocity2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import java.util.function.Supplier
import kotlin.math.PI
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose2dWPILIB
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inMetersPerSecondPerMeter
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
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

class FollowChoreoPath(
    val drivetrain: Drive,
    val trajectory: Trajectory<SwerveSample>,
    val overrideRotationTrigger: Supplier<Boolean> = Supplier { false },
    val flipVertically: Boolean = false,
    val interruptAtTimeout: Boolean = false,
    val extraTime: Time = 0.seconds
) : Command() {

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>
  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  val thetakP =
      LoggedTunableValue(
          "FollowChoreoPath/thetakP",
          Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian }))
  val thetakI =
      LoggedTunableValue(
          "FollowChoreoPath/thetakI",
          Pair(
              { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }))
  val thetakD =
      LoggedTunableValue(
          "FollowChoreoPath/thetakD",
          Pair(
              { it.inRadiansPerSecondPerRadianPerSecond },
              { it.radians.perSecond.perRadianPerSecond }))

  val poskP =
      LoggedTunableValue(
          "FollowChoreoPath/posKP",
          DrivetrainConstants.PID.AUTO_POS_KP,
          Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter }))
  val poskI =
      LoggedTunableValue(
          "FollowChoreoPath/posKI",
          DrivetrainConstants.PID.AUTO_POS_KI,
          Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds }))
  val poskD =
      LoggedTunableValue(
          "FollowChoreoPath/posKD",
          DrivetrainConstants.PID.AUTO_POS_KD,
          Pair(
              { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }))

  private val finalPose: Pose2d =
      applyFlip(Pose2d(trajectory.getFinalPose(AllianceFlipUtil.shouldFlip()).get()))

  val swerveDriveController: CustomHolonomicDriveController

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

    xPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    yPID = PIDController(poskP.get(), poskI.get(), poskD.get())
    thetaPID = PIDController(thetakP.get(), thetakI.get(), thetakD.get())

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    swerveDriveController =
        CustomHolonomicDriveController(
            xPID.wpiPidController, yPID.wpiPidController, thetaPID.wpiPidController)

    swerveDriveController.setTolerance(Pose2d(2.5.inches, 2.5.inches, 5.degrees).pose2d)
  }

  override fun initialize() {
    CustomLogger.recordOutput("ActiveCommands/FollowChoreoPath", true)
    thetaPID.reset()
    xPID.reset()
    yPID.reset()
  }

  override fun execute() {
    if (trajStartTime == 0.seconds) trajStartTime = Clock.timestamp

    trajCurTime = Clock.timestamp - trajStartTime

    val desiredState =
        trajectory.sampleAt(trajCurTime.inSeconds, AllianceFlipUtil.shouldFlip()).get()

    val wantedPose = applyFlip(Pose2d(desiredState.pose))

    CustomLogger.recordOutput("FollowChoreoPath/desiredPose", wantedPose.pose2d)
    CustomLogger.recordOutput("FollowChoreoPath/desiredSpeeds", desiredState.chassisSpeeds)

    val nextDriveState =
        swerveDriveController.calculate(applyFlip(drivetrain.pose).pose2d, desiredState)

    if (overrideRotationTrigger.get())
        drivetrain.runTranslationWhileKeepingRotation(
            Velocity2d(
                nextDriveState.vxMetersPerSecond.meters.perSecond * if (flipVertically) -1 else 1,
                nextDriveState.vyMetersPerSecond.meters.perSecond * if (flipVertically) -1 else 1),
            flipIfRed = false)
    else
        drivetrain.runSpeeds(
            ChassisSpeeds(
                nextDriveState.vxMetersPerSecond.meters.perSecond,
                nextDriveState.vyMetersPerSecond.meters.perSecond * if (flipVertically) -1 else 1,
                nextDriveState.omegaRadiansPerSecond.radians.perSecond *
                    if (flipVertically) -1 else 1),
            flipIfRed = false)

    CustomLogger.recordOutput("FollowChoreoPath/atSetpoint", atSetpoint())
  }

  private fun atSetpoint(): Boolean {
    val posediff = drivetrain.pose.minus(finalPose)

    CustomLogger.recordOutput("FollowChoreoPath/poseDiff", posediff.transform2d)

    CustomLogger.recordOutput(
        "FollowChoreoPath/poseDiffX", posediff.translation.x.absoluteValue < 3.inches)
    CustomLogger.recordOutput(
        "FollowChoreoPath/poseDiffY", posediff.translation.y.absoluteValue < 3.inches)
    CustomLogger.recordOutput(
        "FollowChoreoPath/poseDiffRot", posediff.rotation.absoluteValue < 5.degrees)

    return posediff.translation.x.absoluteValue < 3.inches &&
        posediff.translation.y.absoluteValue < 3.inches &&
        (posediff.rotation.absoluteValue < 5.degrees ||
            posediff.rotation.absoluteValue > 355.degrees)
  }

  fun applyFlip(pose: Pose2d): Pose2d {
    if (!flipVertically) return pose
    return flipVertically(pose)
  }

  override fun isFinished(): Boolean {
    val timedOut = Clock.timestamp - trajStartTime > trajectory.totalTime.seconds + extraTime
    return timedOut && (interruptAtTimeout || atSetpoint()) || !DriverStation.isAutonomous()
  }

  override fun end(interrupted: Boolean) {
    trajStartTime = 0.seconds
    CustomLogger.recordDebugOutput("ActiveCommands/FollowChoreoPath", false)
    drivetrain.runSpeeds(ChassisSpeeds())
  }

  companion object {
    fun flipVertically(pose: Pose2d): Pose2d {
      return Pose2d(pose.x, FieldConstants.fieldWidth - pose.y, -pose.rotation)
    }

    fun warmupCmd(): Command {
      val autoFactory =
          AutoFactory(
              { Pose2dWPILIB() },
              { _: Pose2dWPILIB -> },
              { _: SwerveSample -> },
              false,
              object : Subsystem {})
      return autoFactory.warmupCmd()
    }
  }
}

package com.team4099.robot2025.commands.drivetrain

import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.team4099.lib.logging.LoggedTunableValue
import com.team4099.lib.math.asTransform2d
import com.team4099.lib.trajectory.CustomHolonomicDriveController
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.hal.Clock
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Meter
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
import kotlin.math.PI

class FollowChoreoPath(val drivetrain: Drive, val trajectory: Trajectory<SwerveSample>) :
  Command() {

  private val xPID: PIDController<Meter, Velocity<Meter>>
  private val yPID: PIDController<Meter, Velocity<Meter>>
  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private var trajCurTime = 0.0.seconds
  private var trajStartTime = 0.0.seconds

  val thetakP =
    LoggedTunableValue(
      "Pathfollow/thetakP",
      Pair({ it.inRadiansPerSecondPerRadian }, { it.radians.perSecond.perRadian })
    )
  val thetakI =
    LoggedTunableValue(
      "Pathfollow/thetakI",
      Pair(
        { it.inRadiansPerSecondPerRadianSeconds }, { it.radians.perSecond.perRadianSeconds }
      )
    )
  val thetakD =
    LoggedTunableValue(
      "Pathfollow/thetakD",
      Pair(
        { it.inRadiansPerSecondPerRadianPerSecond },
        { it.radians.perSecond.perRadianPerSecond }
      )
    )

  val poskP =
    LoggedTunableValue(
      "Pathfollow/posKP",
      DrivetrainConstants.PID.AUTO_POS_KP,
      Pair({ it.inMetersPerSecondPerMeter }, { it.meters.perSecond.perMeter })
    )
  val poskI =
    LoggedTunableValue(
      "Pathfollow/posKI",
      DrivetrainConstants.PID.AUTO_POS_KI,
      Pair({ it.inMetersPerSecondPerMeterSeconds }, { it.meters.perSecond.perMeterSeconds })
    )
  val poskD =
    LoggedTunableValue(
      "Pathfollow/posKD",
      DrivetrainConstants.PID.AUTO_POS_KD,
      Pair(
        { it.inMetersPerSecondPerMetersPerSecond }, { it.metersPerSecondPerMetersPerSecond }
      )
    )

  private val finalPose: Pose2d =
    Pose2d(trajectory.getFinalPose(AllianceFlipUtil.shouldFlip()).get())

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
        xPID.wpiPidController, yPID.wpiPidController, thetaPID.wpiPidController
      )

    swerveDriveController.setTolerance(Pose2d(2.5.inches, 2.5.inches, 10.degrees).pose2d)
  }

  override fun initialize() {
    CustomLogger.recordOutput("ActiveCommands/FollowChoreoPath", true)
    thetaPID.reset()
    xPID.reset()
    yPID.reset()
  }

  override fun execute() {
    if (trajStartTime == 0.seconds) trajStartTime = Clock.fpgaTime

    trajCurTime = Clock.fpgaTime - trajStartTime

    val desiredState =
      trajectory.sampleAt(trajCurTime.inSeconds, AllianceFlipUtil.shouldFlip()).get()

    CustomLogger.recordOutput("FollowChoreoPath/desiredPose", desiredState.pose)

    val nextDriveState =
      swerveDriveController.calculate(drivetrain.pose.toPose2d().pose2d, desiredState)
    drivetrain.runSpeeds(
      ChassisSpeeds(
        nextDriveState.vxMetersPerSecond.meters.perSecond,
        nextDriveState.vyMetersPerSecond.meters.perSecond,
        -nextDriveState.omegaRadiansPerSecond.radians.perSecond
      ),
      flipIfRed = false
    )

    CustomLogger.recordOutput("FollowChoreoPath/atSetpoint", atSetpoint())
  }

  private fun atSetpoint(): Boolean {
    val posediff = drivetrain.pose.toPose2d().relativeTo(finalPose)

    CustomLogger.recordOutput("FollowChoreoPath/poseDiff", posediff.asTransform2d().transform2d)

    return posediff.x.absoluteValue < 3.inches &&
      posediff.y.absoluteValue < 3.inches &&
      posediff.rotation.absoluteValue < 5.degrees
  }

  override fun isFinished(): Boolean {
    return Clock.fpgaTime - trajStartTime > trajectory.totalTime.seconds && atSetpoint()
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordDebugOutput("ActiveCommands/FollowChoreoPath", false)
    drivetrain.runSpeeds(ChassisSpeeds())
  }
}

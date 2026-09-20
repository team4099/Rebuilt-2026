package com.team4099.robot2026.auto

import com.team4099.robot2026.auto.mode.ExamplePathAuto
import com.team4099.robot2026.auto.mode.IntakeQuadrantFollowClose
import com.team4099.robot2026.auto.mode.IntakeQuadrantFollowFar
import com.team4099.robot2026.auto.mode.IntakeQuadrantL1
import com.team4099.robot2026.auto.mode.PreloadL1Auto
import com.team4099.robot2026.auto.mode.TuningAutoPos
import com.team4099.robot2026.commands.characterization.DriveCharacterizationCommands
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import edu.wpi.first.math.geometry.Pose2d as WPILibPose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds as WPILIBSpeeds
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.BLineCommands
import frc.robot.lib.BLine.FollowPath
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.inRadiansPerSecond

class AutonomousSelector(val drivetrain: Drive) {
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
      LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry
  private var fieldSideChooser: LoggedDashboardChooser<Boolean> =
      LoggedDashboardChooser("FieldSide")

  private val pathBuilder: FollowPath.Builder

  private val autoAimOTFCommand = AimOTFCommand(drivetrain, timeout = 20.seconds)

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")

    fieldSideChooser.addOption("Left", false)
    fieldSideChooser.addOption("Right", true)

    autonomousModeChooser.addOption(
        "Example Auto DO NOT RUN AT COMPETITION", AutonomousMode.EXAMPLE_AUTO)
    autonomousModeChooser.addOption(
        "WheelRadius DO NOT RUN AT COMPETITION", AutonomousMode.WHEEL_RADIUS)
    autonomousModeChooser.addOption(
        "Drive FF Characterization DO NOT RUN AT COMPETITION", AutonomousMode.DRIVE_FF)
    autonomousModeChooser.addOption(
        "Auto Pose Tuner DO NOT RUN AT COMPETITION", AutonomousMode.AUTOPOS)
    autonomousModeChooser.addOption("Intake Quadrant L1", AutonomousMode.INTAKE_QUAD_L1)
    autonomousModeChooser.addOption("Preload + Bump Center", AutonomousMode.PRELOAD_BUMP_CENTER)
    autonomousModeChooser.addOption(
        "Intake Follow Close (ADD A WAIT TIME)", AutonomousMode.INTAKE_FOLLOW_CLOSE)
    autonomousModeChooser.addOption(
        "Intake Follow Far (ADD A WAIT TIME)", AutonomousMode.INTAKE_FOLLOW_FAR)
    autonomousModeChooser.addOption("Do nothing", AutonomousMode.DO_NOTHING)

    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(4, 2).withPosition(2, 0)

    waitBeforeCommandSlider =
        autoTab
            .add("Wait Time", 0)
            .withSize(3, 2)
            .withPosition(0, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .entry

    pathBuilder =
        FollowPath.Builder(
                drivetrain,
                { drivetrain.pose.pose2d },
                { drivetrain.chassisSpeeds.chassisSpeedsWPILIB },
                { speeds: WPILIBSpeeds ->
                  drivetrain.runSpeeds(ChassisSpeeds(speeds), flipIfRed = false)
                },
                PIDController(
                        DrivetrainConstants.PID.AUTO_POS_KP,
                        DrivetrainConstants.PID.AUTO_POS_KI,
                        DrivetrainConstants.PID.AUTO_POS_KD)
                    .wpiPidController,
                PIDController(
                        DrivetrainConstants.PID.AUTO_THETA_PID_KP,
                        DrivetrainConstants.PID.AUTO_THETA_PID_KI,
                        DrivetrainConstants.PID.AUTO_THETA_PID_KD)
                    .wpiPidController,
                PIDController(
                        DrivetrainConstants.PID.AUTO_CROSSTRACK_KP,
                        DrivetrainConstants.PID.AUTO_CROSSTRACK_KI,
                        DrivetrainConstants.PID.AUTO_CROSSTRACK_KD)
                    .wpiPidController)
            .withDefaultShouldFlip()
            .withTRatioBasedTranslationHandoffs(true)
            .withShouldMirror { fieldSideChooser.get() ?: false }
            .withPoseReset { startingPose: WPILibPose2d -> drivetrain.pose = Pose2d(startingPose) }
  }

  fun registerEventTriggers(superstructure: Superstructure) {
    FollowPath.registerEventTrigger("startIntaking", superstructure.requestIntakeCommand())
    FollowPath.registerEventTrigger("stopIntaking", superstructure.requestIdleCommand())
    FollowPath.registerEventTrigger("startShooting", superstructure.requestScoreCommand())
    FollowPath.registerEventTrigger("stopShooting", superstructure.requestIdleCommand())
    FollowPath.registerEventTrigger(
        "startAiming",
        Commands.runOnce({
              FollowPath.overrideRotation(
                  { autoAimOTFCommand.omega.inRadiansPerSecond },
                  FollowPath.RotationOverrideBehavior.BYPASS_CONSTRAINTS)
            })
            .andThen(BLineCommands.deferredProxy { autoAimOTFCommand }))
    FollowPath.registerEventTrigger(
        "stopAiming", Commands.runOnce({ FollowPath.clearRotationOverride() }))
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  fun getCommand(superstructure: Superstructure, intake: Intake): Command {
    val mode = autonomousModeChooser.get()

    pathBuilder.withPoseReset { startingPose: WPILibPose2d ->
      drivetrain.pose = Pose2d(startingPose)
    }

    val command =
        when (mode) {
          AutonomousMode.EXAMPLE_AUTO -> ExamplePathAuto(drivetrain, pathBuilder)
          AutonomousMode.WHEEL_RADIUS ->
              DriveCharacterizationCommands.wheelRadiusCharacterization(drivetrain)
          AutonomousMode.DRIVE_FF ->
              DriveCharacterizationCommands.feedforwardCharacterization(drivetrain)
          AutonomousMode.AUTOPOS -> TuningAutoPos(drivetrain, pathBuilder)
          AutonomousMode.INTAKE_QUAD_L1 ->
              IntakeQuadrantL1(drivetrain, superstructure, intake, pathBuilder).finallyDo { _ ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.PRELOAD_BUMP_CENTER ->
              PreloadL1Auto(drivetrain, superstructure, intake, pathBuilder).finallyDo { _ ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.INTAKE_FOLLOW_CLOSE ->
              IntakeQuadrantFollowClose(drivetrain, superstructure, intake, pathBuilder)
                  .finallyDo { _ -> FollowPath.clearRotationOverride() }
          AutonomousMode.INTAKE_FOLLOW_FAR ->
              IntakeQuadrantFollowFar(drivetrain, superstructure, intake, pathBuilder).finallyDo { _
                ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.DO_NOTHING -> InstantCommand()
          else -> InstantCommand()
        }

    // note(nathan): saves one loop
    return if (waitTime != 0.seconds) command else WaitCommand(waitTime.inSeconds).andThen(command)
  }
}

private enum class AutonomousMode {
  EXAMPLE_AUTO,
  WHEEL_RADIUS,
  DRIVE_FF,
  AUTOPOS,
  INTAKE_QUAD_L1,
  PRELOAD_BUMP_CENTER,
  INTAKE_FOLLOW_CLOSE,
  INTAKE_FOLLOW_FAR,
  DO_NOTHING,
}

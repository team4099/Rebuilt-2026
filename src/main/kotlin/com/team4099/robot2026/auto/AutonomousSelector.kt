package com.team4099.robot2026.auto

import com.team4099.robot2026.auto.mode.BigCircle
import com.team4099.robot2026.auto.mode.ExamplePathAuto
import com.team4099.robot2026.auto.mode.IntakeQuadrantFollowClose
import com.team4099.robot2026.auto.mode.IntakeQuadrantL1
import com.team4099.robot2026.auto.mode.IntakeSideSpin
import com.team4099.robot2026.auto.mode.PreloadL1Auto
import com.team4099.robot2026.auto.mode.TestOTFAuto
import com.team4099.robot2026.auto.mode.TestingAuto
import com.team4099.robot2026.auto.mode.TuningAutoPos
import com.team4099.robot2026.commands.characterization.DriveCharacterizationCommands
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import com.team4099.robot2026.util.AllianceFlipUtil
import edu.wpi.first.math.geometry.Pose2d as WPILibPose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds as WPILIBSpeeds
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.FollowPath
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

class AutonomousSelector(val drivetrain: Drive) {
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
      LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry
  private var fieldSideChooser: LoggedDashboardChooser<Boolean> =
      LoggedDashboardChooser("FieldSide")

  private val pathBuilder: FollowPath.Builder

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
    autonomousModeChooser.addOption("TestOTF DO NOT RUN AT COMPETITION", AutonomousMode.TEST_OTF)
    autonomousModeChooser.addOption(
        "Auto Pose Tuner DO NOT RUN AT COMPETITION", AutonomousMode.AUTOPOS)
    autonomousModeChooser.addOption("BIG CIRCLE AUTO RUN ONLY AT 836", AutonomousMode.BIG_CIRCLE)
    autonomousModeChooser.addOption(
        "Miscellaneous Testing Auto DO NOT RUN AT COMPETITION", AutonomousMode.TESTING)
    autonomousModeChooser.addOption("Intake Right Quadrant L1", AutonomousMode.INTAKE_RIGHT_QUAD_L1)
    autonomousModeChooser.addOption("Intake Left Quadrant L1", AutonomousMode.INTAKE_LEFT_QUAD_L1)
    autonomousModeChooser.addOption("Intake Right Spin", AutonomousMode.INTAKE_RIGHT_SPIN)
    autonomousModeChooser.addOption("Intake Left Spin", AutonomousMode.INTAKE_LEFT_SPIN)
    //  autonomousModeChooser.addOption("Centerline Sweep Left",
    // AutonomousMode.CENTERLINE_SWEEP_LEFT)
    // autonomousModeChooser.addOption("Centerline Sweep Right",
    // AutonomousMode.CENTERLINE_SWEEP_RIGHT)
    autonomousModeChooser.addOption(
        "Preload + Left Bump Center", AutonomousMode.PRELOAD_BUMP_CENTER_LEFT)
    autonomousModeChooser.addOption(
        "Preload + Right Bump Center", AutonomousMode.PRELOAD_BUMP_CENTER_RIGHT)
    autonomousModeChooser.addOption(
        "Intake Follow Close Right (ADD A WAIT TIME)", AutonomousMode.INTAKE_FOLLOW_CLOSE_RIGHT)
    autonomousModeChooser.addOption(
        "Intake Follow Close Left (ADD A WAIT TIME)", AutonomousMode.INTAKE_FOLLOW_CLOSE_LEFT)
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

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  fun getCommand(superstructure: Superstructure, intake: Intake): Command {
    val mode = autonomousModeChooser.get()

    return when (mode) {
      AutonomousMode.EXAMPLE_AUTO ->
          WaitCommand(waitTime.inSeconds).andThen(ExamplePathAuto(drivetrain, pathBuilder))
      AutonomousMode.WHEEL_RADIUS ->
          DriveCharacterizationCommands.wheelRadiusCharacterization(drivetrain)
      AutonomousMode.DRIVE_FF ->
          DriveCharacterizationCommands.feedforwardCharacterization(drivetrain)
      AutonomousMode.TEST_OTF ->
          WaitCommand(waitTime.inSeconds)
              .andThen({ drivetrain.pose = AllianceFlipUtil.apply(TestOTFAuto.startingPose) })
              .andThen(TestOTFAuto(drivetrain))
      AutonomousMode.AUTOPOS ->
          WaitCommand(waitTime.inSeconds)
              .andThen({ drivetrain.pose = AllianceFlipUtil.apply(TuningAutoPos.startingPose) })
              .andThen(TuningAutoPos(drivetrain))
      AutonomousMode.TESTING ->
          WaitCommand(waitTime.inSeconds).andThen(TestingAuto(drivetrain, superstructure))
      AutonomousMode.INTAKE_RIGHT_QUAD_L1 ->
          WaitCommand(waitTime.inSeconds)
              .andThen({ drivetrain.pose = AllianceFlipUtil.apply(IntakeQuadrantL1.startingPose) })
              .andThen(IntakeQuadrantL1(drivetrain, superstructure, intake, flipVeritcally = false))
      AutonomousMode.INTAKE_LEFT_QUAD_L1 ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose =
                    FollowChoreoPath.flipVertically(
                        AllianceFlipUtil.apply(IntakeQuadrantL1.startingPose))
              })
              .andThen(IntakeQuadrantL1(drivetrain, superstructure, intake, flipVeritcally = true))
      AutonomousMode.INTAKE_RIGHT_SPIN ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose = Pose2d(AllianceFlipUtil.apply(IntakeSideSpin.startingPose).pose2d)
              })
              .andThen(IntakeSideSpin(drivetrain, superstructure, intake, flipVeritcally = false))
      AutonomousMode.INTAKE_LEFT_SPIN ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose =
                    Pose2d(
                        FollowChoreoPath.flipVertically(
                                AllianceFlipUtil.apply(IntakeSideSpin.startingPose))
                            .pose2d)
              })
              .andThen(IntakeSideSpin(drivetrain, superstructure, intake, flipVeritcally = true))
      //      AutonomousMode.CENTERLINE_SWEEP_RIGHT ->
      //          WaitCommand(waitTime.inSeconds)
      //              .andThen({
      //                drivetrain.pose =
      // Pose3d(AllianceFlipUtil.apply(CenterlineSweep.startingPose))
      //              })
      //              .andThen(CenterlineSweep(drivetrain, superstructure, flipVeritcally = true))
      //      AutonomousMode.CENTERLINE_SWEEP_LEFT ->
      //          WaitCommand(waitTime.inSeconds)
      //              .andThen({
      //                drivetrain.pose =
      //                    Pose3d(
      //                        FollowChoreoPath.flipVertically(
      //                            AllianceFlipUtil.apply(CenterlineSweep.startingPose)))
      //              })
      //              .andThen(CenterlineSweep(drivetrain, superstructure, flipVeritcally = false))
      AutonomousMode.PRELOAD_BUMP_CENTER_LEFT ->
          WaitCommand(waitTime.inSeconds)
              .andThen({ drivetrain.pose = AllianceFlipUtil.apply(PreloadL1Auto.startingPose) })
              .andThen(PreloadL1Auto(drivetrain, superstructure, false))
      AutonomousMode.PRELOAD_BUMP_CENTER_RIGHT ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose =
                    Pose2d(
                        FollowChoreoPath.flipVertically(
                                AllianceFlipUtil.apply(PreloadL1Auto.startingPose))
                            .pose2d)
              })
              .andThen(PreloadL1Auto(drivetrain, superstructure, true))
      AutonomousMode.INTAKE_FOLLOW_CLOSE_RIGHT ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose =
                    Pose2d(AllianceFlipUtil.apply(IntakeQuadrantFollowClose.startingPose).pose2d)
              })
              .andThen(IntakeQuadrantFollowClose(drivetrain, superstructure, intake, false))
      AutonomousMode.INTAKE_FOLLOW_CLOSE_LEFT ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose =
                    Pose2d(
                        FollowChoreoPath.flipVertically(
                                AllianceFlipUtil.apply(IntakeQuadrantFollowClose.startingPose))
                            .pose2d)
              })
              .andThen(IntakeQuadrantFollowClose(drivetrain, superstructure, intake, true))
      AutonomousMode.BIG_CIRCLE ->
          WaitCommand(waitTime.inSeconds)
              .andThen({ drivetrain.pose = AllianceFlipUtil.apply(BigCircle.startingPose) })
              .andThen(BigCircle(drivetrain))
      AutonomousMode.DO_NOTHING -> InstantCommand()
      else -> InstantCommand()
    }
  }
}

private enum class AutonomousMode {
  EXAMPLE_AUTO,
  WHEEL_RADIUS,
  TEST_OTF,
  DRIVE_FF,
  AUTOPOS,
  TESTING,
  BIG_CIRCLE,
  INTAKE_RIGHT_QUAD_L1,
  INTAKE_LEFT_QUAD_L1,
  INTAKE_RIGHT_SPIN,
  INTAKE_LEFT_SPIN,
  CENTERLINE_SWEEP_RIGHT,
  CENTERLINE_SWEEP_LEFT,
  PRELOAD_BUMP_CENTER_LEFT,
  PRELOAD_BUMP_CENTER_RIGHT,
  INTAKE_FOLLOW_CLOSE_RIGHT,
  INTAKE_FOLLOW_CLOSE_LEFT,
  DO_NOTHING,
}

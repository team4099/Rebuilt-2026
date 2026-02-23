package com.team4099.robot2026.auto

import com.team4099.robot2026.auto.mode.CenterShootClimb
import com.team4099.robot2026.auto.mode.ExamplePathAuto
import com.team4099.robot2026.auto.mode.IntakeQuadrantL1
import com.team4099.robot2026.auto.mode.TestOTFAuto
import com.team4099.robot2026.auto.mode.TuningAutoPos
import com.team4099.robot2026.commands.characterization.DriveCharacterizationCommands
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.AllianceFlipUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds

object AutonomousSelector {
  private var autonomousModeChooser: LoggedDashboardChooser<AutonomousMode> =
      LoggedDashboardChooser("AutonomousMode")
  private var waitBeforeCommandSlider: GenericEntry

  init {
    val autoTab = Shuffleboard.getTab("Pre-match")

    autonomousModeChooser.addOption(
        "Example Auto DO NOT RUN AT COMPETITION", AutonomousMode.EXAMPLE_AUTO)
    autonomousModeChooser.addOption("Center intake OFT shoot and climb", AutonomousMode.OTF_AUTO)
    //    autonomousModeChooser.addOption("WheelRadius DO NOT RUN AT COMPETITION",
    // AutonomousMode.WHEEL_RADIUS)
    autonomousModeChooser.addOption(
        "WheelRadius DO NOT RUN AT COMPETITION", AutonomousMode.WHEEL_RADIUS)
    autonomousModeChooser.addOption(
        "Drive FF Characterization DO NOT RUN AT COMPETITION", AutonomousMode.DRIVE_FF)
    autonomousModeChooser.addOption("TestOTF DO NOT RUN AT COMPETITION", AutonomousMode.TEST_OTF)
    autonomousModeChooser.addOption(
        "Auto Pose Tuner DO NOT RUN AT COMPETITION", AutonomousMode.AUTOPOS)
    autonomousModeChooser.addOption("Intake Right Quadrant L1", AutonomousMode.INTAKE_RIGHT_QUAD_L1)
    autonomousModeChooser.addOption("Intake Left Quadrant L1", AutonomousMode.INTAKE_LEFT_QUAD_L1)
    autonomousModeChooser.addOption("Do nothing", AutonomousMode.DO_NOTHING)

    autoTab.add("Mode", autonomousModeChooser.sendableChooser).withSize(4, 2).withPosition(2, 0)

    waitBeforeCommandSlider =
        autoTab
            .add("Wait Time Before Shooting", 0)
            .withSize(3, 2)
            .withPosition(0, 2)
            .withWidget(BuiltInWidgets.kTextView)
            .entry
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.getDouble(0.0).seconds

  fun getCommand(drivetrain: Drive, vision: Vision,shooter: Shooter, superstructure: Superstructure): Command {
    val mode = autonomousModeChooser.get()

    return when (mode) {
      AutonomousMode.EXAMPLE_AUTO ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose = Pose3d(AllianceFlipUtil.apply(ExamplePathAuto.startingPose))
              })
              .andThen(ExamplePathAuto(drivetrain))
      AutonomousMode.WHEEL_RADIUS ->
          DriveCharacterizationCommands.wheelRadiusCharacterization(drivetrain)
      AutonomousMode.DRIVE_FF ->
          DriveCharacterizationCommands.feedforwardCharacterization(drivetrain)
      AutonomousMode.TEST_OTF ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose = Pose3d(AllianceFlipUtil.apply(TestOTFAuto.startingPose))
              })
              .andThen(TestOTFAuto(drivetrain))
      AutonomousMode.AUTOPOS ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose = Pose3d(AllianceFlipUtil.apply(TuningAutoPos.startingPose))
              })
              .andThen(TuningAutoPos(drivetrain))
      AutonomousMode.OTF_AUTO ->
          return WaitCommand(waitTime.inSeconds).andThen(CenterShootClimb(drivetrain, shooter))
      AutonomousMode.INTAKE_RIGHT_QUAD_L1 ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose = Pose3d(AllianceFlipUtil.apply(IntakeQuadrantL1.startingPose))
              })
              .andThen(IntakeQuadrantL1(drivetrain, superstructure, flipVeritcally = false))
      AutonomousMode.INTAKE_LEFT_QUAD_L1 ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose =
                    Pose3d(
                        FollowChoreoPath.flipVertically(
                            AllianceFlipUtil.apply(IntakeQuadrantL1.startingPose)))
              })
              .andThen(IntakeQuadrantL1(drivetrain, superstructure, flipVeritcally = true))
      AutonomousMode.DO_NOTHING -> InstantCommand()
      else -> InstantCommand()
    }
  }
}

private enum class AutonomousMode {
  EXAMPLE_AUTO,
  OTF_AUTO,
  WHEEL_RADIUS,
  TEST_OTF,
  DRIVE_FF,
  AUTOPOS,
  INTAKE_RIGHT_QUAD_L1,
  INTAKE_LEFT_QUAD_L1,
  DO_NOTHING
}

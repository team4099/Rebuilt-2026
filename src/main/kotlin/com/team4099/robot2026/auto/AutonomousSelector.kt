package com.team4099.robot2026.auto

import com.team4099.robot2026.auto.mode.ExamplePathAuto
import com.team4099.robot2026.auto.mode.SysID
import com.team4099.robot2026.auto.mode.TestOTFAuto
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.util.AllianceFlipUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.DriveCharacterizationCommands
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
    autonomousModeChooser.addOption("SysID DO NOT RUN AT COMPETITION", AutonomousMode.SYSID)
    autonomousModeChooser.addOption(
        "WheelRadius DO NOT RUN AT COMPETITION", AutonomousMode.WHEEL_RADIUS)
    autonomousModeChooser.addOption("TestOTF DO NOT RUN AT COMPETITION", AutonomousMode.TEST_OTF)

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

  fun getCommand(drivetrain: Drive, vision: Vision): Command {
    val mode = autonomousModeChooser.get()

    return when (mode) {
      AutonomousMode.EXAMPLE_AUTO ->
          WaitCommand(waitTime.inSeconds)
            .andThen({
              drivetrain.pose = Pose3d(AllianceFlipUtil.apply(ExamplePathAuto.startingPose))
            })
            .andThen(ExamplePathAuto(drivetrain))
      AutonomousMode.SYSID -> WaitCommand(waitTime.inSeconds).andThen(SysID(drivetrain))
      AutonomousMode.WHEEL_RADIUS ->
          DriveCharacterizationCommands.wheelRadiusCharacterization(drivetrain)
      AutonomousMode.TEST_OTF ->
          WaitCommand(waitTime.inSeconds)
              .andThen({
                drivetrain.pose = Pose3d(AllianceFlipUtil.apply(TestOTFAuto.startingPose))
              })
              .andThen(TestOTFAuto(drivetrain))
      else -> InstantCommand()
    }
  }
}

private enum class AutonomousMode {
  EXAMPLE_AUTO,
  SYSID,
  WHEEL_RADIUS,
  TEST_OTF
}

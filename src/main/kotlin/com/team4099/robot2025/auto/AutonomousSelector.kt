package com.team4099.robot2025.auto

import com.team4099.robot2025.auto.mode.ExamplePathAuto
import com.team4099.robot2025.auto.mode.SysID
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.vision.Vision
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
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
      "Example Auto DO NOT RUN AT COMPETITION", AutonomousMode.EXAMPLE_AUTO
    )
    autonomousModeChooser.addOption("SysID DON'T RUN AT COMP", AutonomousMode.SYSID)

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

    when (mode) {
      AutonomousMode.EXAMPLE_AUTO ->
        return WaitCommand(waitTime.inSeconds).andThen(ExamplePathAuto(drivetrain))
      AutonomousMode.SYSID -> return WaitCommand(waitTime.inSeconds).andThen(SysID(drivetrain))
      else -> return InstantCommand()
    }
  }
}

private enum class AutonomousMode {
  EXAMPLE_AUTO,
  SYSID
}

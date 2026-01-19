package com.team4099.robot2025.auto.mode

import com.team4099.robot2025.commands.characterization.FeedforwardCharacterizationCommand
import com.team4099.robot2025.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class SysID(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)
    addCommands(WaitCommand(0.5), FeedforwardCharacterizationCommand(drivetrain))
  }

  companion object {}
}

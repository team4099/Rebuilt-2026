package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.commands.characterization.WheelRadiusCharacterizationCommand
import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class WheelRadius(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addCommands(WaitCommand(0.5), WheelRadiusCharacterizationCommand(drivetrain))
  }
}

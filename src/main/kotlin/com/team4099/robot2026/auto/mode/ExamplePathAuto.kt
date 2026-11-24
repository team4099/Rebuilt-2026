package com.team4099.robot2026.auto.mode

import bline.path.Path
import com.team4099.robot2026.auto.AutoPathBuilder
import com.team4099.robot2026.subsystems.drivetrain.Drive
import org.wpilib.command2.SequentialCommandGroup
import org.wpilib.command2.WaitCommand

class ExamplePathAuto(val drivetrain: Drive, pathBuilder: AutoPathBuilder) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(WaitCommand(0.5), pathBuilder.build(pathOne))
  }

  companion object {
    val pathOne = Path("straightline")
  }
}

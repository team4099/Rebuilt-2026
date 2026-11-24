package com.team4099.robot2026.auto.mode

import bline.path.Path
import com.team4099.robot2026.auto.AutoPathBuilder
import com.team4099.robot2026.subsystems.drivetrain.Drive
import org.wpilib.command2.SequentialCommandGroup

class TuningAutoPos(val drivetrain: Drive, pathBuilder: AutoPathBuilder) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(pathBuilder.build(traj))
  }

  companion object {
    val traj = Path("tuning")
  }
}

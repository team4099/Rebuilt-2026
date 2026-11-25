package com.team4099.robot2026.auto.mode

import bline.commands.FollowPathV2
import bline.path.Path
import com.team4099.robot2026.subsystems.drivetrain.Drive
import org.wpilib.command2.SequentialCommandGroup

class TuningAutoPos(val drivetrain: Drive, pathBuilder: (Path, Boolean) -> FollowPathV2) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(pathBuilder(traj, true))
  }

  companion object {
    val traj = Path("tuning")
  }
}

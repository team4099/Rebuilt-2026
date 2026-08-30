package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path

class TuningAutoPos(val drivetrain: Drive, pathBuilder: FollowPath.Builder) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(pathBuilder.build(traj))
  }

  companion object {
    val traj = Path("tuning")
  }
}

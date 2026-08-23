package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.math.geometry.Pose2d as WPILIBPose2d
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import org.team4099.lib.geometry.Pose2d

class ExamplePathAuto(val drivetrain: Drive, val pathBuilder: FollowPath.Builder) :
    SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
        WaitCommand(0.5),
        pathBuilder
            .withPoseReset { startingPose: WPILIBPose2d -> drivetrain.pose = Pose2d(startingPose) }
            .build(pathOne))
  }

  companion object {
    val pathOne = Path("straightline")
  }
}

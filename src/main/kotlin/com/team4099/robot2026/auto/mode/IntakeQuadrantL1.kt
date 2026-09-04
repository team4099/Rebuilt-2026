package com.team4099.robot2026.auto.mode


import com.team4099.robot2026.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path


class IntakeQuadrantL1(val drivetrain: Drive, pathBuilder: FollowPath.Builder) :
  SequentialCommandGroup() {
  init {
    addCommands(pathBuilder.build(pathOne))
  }

  companion object {
    val pathOne = Path("intakequadl1")
  }
}

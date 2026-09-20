package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.commands.AgitateIntakeCommand
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.robot.lib.BLine.BLineCommands
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path

class PreloadL1Auto(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val intake: Intake,
    pathBuilder: FollowPath.Builder
) :
    WrapperCommand(
        BLineCommands.sequence(
            pathBuilder.build(pathOne),
            WaitCommand(5.0),
            Commands.runOnce({ pathBuilder.withPoseReset { _ -> {} } }),
            pathBuilder.build(pathTwo),
            WaitCommand(1.0),
            AgitateIntakeCommand(superstructure, intake).withTimeout(18.0 - (0.90 + 5.0 + 5.62)),
            superstructure.requestForceIntakeCommand(IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE))) {
  companion object {
    val pathOne = Path("preload")
    val pathTwo = Path("preload2quadrant")
  }
}

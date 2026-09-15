package com.team4099.robot2026.auto.mode

import com.team4099.robot2026.commands.AgitateIntakeCommand
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WrapperCommand
import frc.robot.lib.BLine.BLineCommands
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import org.team4099.lib.units.base.seconds

class IntakeQuadrantL1(
    val drivetrain: Drive,
    val superstructure: Superstructure,
    val intake: Intake,
    pathBuilder: FollowPath.Builder
) :
    WrapperCommand(
        BLineCommands.sequence(
            pathBuilder.build(pathOne),
            ParallelCommandGroup(
                AimOTFCommand(drivetrain, 20.seconds),
                SequentialCommandGroup(
                    WaitCommand(1.0),
                    AgitateIntakeCommand(superstructure, intake).withTimeout(18.0 - (4.25 + 1.0)),
                    superstructure.requestForceIntakeCommand(
                        IntakeConstants.ANGLES.FORCE_HALFUP_ANGLE))))) {
  companion object {
    val pathOne = Path("intakequadl1")
  }
}

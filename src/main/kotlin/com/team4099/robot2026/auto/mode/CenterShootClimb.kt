package com.team4099.robot2026.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2026.RobotContainer.superstructure
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.util.AllianceFlipUtil
import com.team4099.robot2026.util.driver.Jessika
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.smoothDeadband

class CenterShootClimb(val drivetrain: Drive, val shooter: Shooter) : ParallelCommandGroup() {
  init {
//    addRequirements(drivetrain)

    addCommands(
        Commands.runOnce({drivetrain.pose = Pose3d(AllianceFlipUtil.apply(startPose)) }),
        FollowChoreoPath(drivetrain, traj),
        SequentialCommandGroup(
            WaitCommand(1.0)
                .andThen(
                    superstructure.requestIntakeCommand(),
                ),
            WaitCommand(1.5)
                .andThen(
                    superstructure.requestForceIntakeUpCommand(),
                ),
            WaitCommand(0.5).andThen(superstructure.requestPrepScoreCommand()),
            WaitCommand(0.5)
                .andThen(
                    AimOTFCommand(
                        drivetrain,
                        {
                          ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
                        },
                        {
                          ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND)
                        },
                        { false },
                        Jessika())),
            WaitUntilCommand { shooter.isAtTargetedVelocity }
                .andThen(
                    superstructure.requestScoreCommand(),
                )))
      WaitCommand(2.0)
      Request.SuperstructureRequest.ExtendClimb()
      Request.SuperstructureRequest.Idle()
      WaitCommand(1.0)
      Request.SuperstructureRequest.RetractClimb()
  }

  companion object {
    val traj = Choreo.loadTrajectory<SwerveSample>("CenterInt_Shoot_Climb/CenterIntakeMiddleClimb.traj").get()
    val startPose = Pose2d(traj.getInitialPose(false).get())
  }
}

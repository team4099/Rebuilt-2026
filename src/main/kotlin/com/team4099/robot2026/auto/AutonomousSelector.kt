package com.team4099.robot2026.auto

import bline.commands.BLineCommandsV2 as BLineCommands
import bline.commands.FollowPathV2 as FollowPath
import bline.following.DriveType
import com.team4099.robot2026.auto.mode.EjectSwipe
import com.team4099.robot2026.auto.mode.ExamplePathAuto
import com.team4099.robot2026.auto.mode.IntakeQuadrantFollowClose
import com.team4099.robot2026.auto.mode.IntakeQuadrantFollowFar
import com.team4099.robot2026.auto.mode.IntakeQuadrantL1
import com.team4099.robot2026.auto.mode.IntakeQuadrantSteal
import com.team4099.robot2026.auto.mode.PreloadL1Auto
import com.team4099.robot2026.auto.mode.TuningAutoPos
import com.team4099.robot2026.commands.characterization.DriveCharacterizationCommands
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.kinematics.ChassisVelocities
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.inRadiansPerSecond
import org.wpilib.backend.NetworkTablesTunableBackend
import org.wpilib.command2.Command
import org.wpilib.command2.Commands
import org.wpilib.command2.InstantCommand
import org.wpilib.command2.WaitCommand
import org.wpilib.math.kinematics.ChassisVelocities as WPILIBSpeeds
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.tunable.TunableDouble
import org.wpilib.tunable.TunableRegistry
import org.wpilib.tunable.Tunables
import org.wpilib.util.Alert

class AutonomousSelector(val drivetrain: Drive) {
  private var autonomousModeChooser: LoggedNetworkChooser<AutonomousMode> =
      LoggedNetworkChooser("/SmartDashboard/AutonomousMode")
  private val waitBeforeCommandSlider: TunableDouble
  private var fieldSideChooser: LoggedNetworkChooser<Boolean> =
      LoggedNetworkChooser("/SmartDashboard/FieldSide")

  private val pathBuilder: (bline.path.Path) -> FollowPath

  private val autoAimOTFCommand = AimOTFCommand(drivetrain, timeout = 20.seconds)

  private val autoSelectionAlert =
      Alert("autoNotSelected", "No auto selected! Please select an auto.", Alert.Level.HIGH)
  private val autoSideAlert =
      Alert("autoSideNotSelected", "No auto side selected! Please select a side.", Alert.Level.HIGH)

  init {
    fieldSideChooser.add("Left", false)
    fieldSideChooser.add("Right", true)

    autonomousModeChooser.add("Example Auto DO NOT RUN AT COMPETITION", AutonomousMode.EXAMPLE_AUTO)
    autonomousModeChooser.add("WheelRadius DO NOT RUN AT COMPETITION", AutonomousMode.WHEEL_RADIUS)
    autonomousModeChooser.add(
        "Drive FF Characterization DO NOT RUN AT COMPETITION", AutonomousMode.DRIVE_FF)
    autonomousModeChooser.add("Auto Pose Tuner DO NOT RUN AT COMPETITION", AutonomousMode.AUTOPOS)
    autonomousModeChooser.add("Intake Quadrant L1", AutonomousMode.INTAKE_QUAD_L1)
    autonomousModeChooser.add("Intake Quadrant Steal", AutonomousMode.INTAKE_QUAD_STEAL)
    autonomousModeChooser.add("Preload + Bump Center", AutonomousMode.PRELOAD_BUMP_CENTER)
    autonomousModeChooser.add(
        "Intake Follow Close (ADD A WAIT TIME)", AutonomousMode.INTAKE_FOLLOW_CLOSE)
    autonomousModeChooser.add(
        "Intake Follow Far (ADD A WAIT TIME)", AutonomousMode.INTAKE_FOLLOW_FAR)
    autonomousModeChooser.add("Intake Eject Full Swipe Shoot", AutonomousMode.EJECT_SWIPE)
    autonomousModeChooser.add("Do nothing", AutonomousMode.DO_NOTHING)

    TunableRegistry.registerBackend(
        "/Pre-match",
        NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/Shuffleboard"))
    waitBeforeCommandSlider = Tunables.addDouble("/Pre-match/Wait Time", 0.0)

    val builder =
        FollowPath.Builder(
                DriveType.SWERVE,
                drivetrain,
                { drivetrain.pose.pose2d },
                { startingPose -> drivetrain.pose = Pose2d(startingPose) },
                { drivetrain.chassisSpeeds.chassisVelocitiesWPILIB },
                { speeds: WPILIBSpeeds ->
                  drivetrain.runSpeeds(ChassisVelocities(speeds), flipIfRed = false)
                },
                PIDController(
                        DrivetrainConstants.PID.AUTO_POS_KP,
                        DrivetrainConstants.PID.AUTO_POS_KI,
                        DrivetrainConstants.PID.AUTO_POS_KD)
                    .wpiPidController,
                PIDController(
                        DrivetrainConstants.PID.AUTO_THETA_PID_KP,
                        DrivetrainConstants.PID.AUTO_THETA_PID_KI,
                        DrivetrainConstants.PID.AUTO_THETA_PID_KD)
                    .wpiPidController,
                PIDController(
                        DrivetrainConstants.PID.AUTO_CROSSTRACK_KP,
                        DrivetrainConstants.PID.AUTO_CROSSTRACK_KI,
                        DrivetrainConstants.PID.AUTO_CROSSTRACK_KD)
                    .wpiPidController)
            .withDefaultShouldFlip()
    pathBuilder = { path ->
      builder.build(path).withShouldMirror { fieldSideChooser.get() ?: false }
    }
  }

  fun registerEventTriggers(superstructure: Superstructure) {
    FollowPath.registerEventTrigger("startIntaking", superstructure.requestIntakeCommand())
    FollowPath.registerEventTrigger("stopIntaking", superstructure.requestIdleCommand())
    FollowPath.registerEventTrigger("startShooting", superstructure.requestScoreCommand())
    FollowPath.registerEventTrigger("stopShooting", superstructure.requestIdleCommand())
    FollowPath.registerEventTrigger(
        "startAiming",
        Commands.runOnce({
              FollowPath.overrideRotation(
                  { autoAimOTFCommand.omega.inRadiansPerSecond },
                  FollowPath.RotationOverrideBehavior.BYPASS_CONSTRAINTS)
            })
            .andThen(BLineCommands.deferredProxy { autoAimOTFCommand }))
    FollowPath.registerEventTrigger(
        "stopAiming", Commands.runOnce({ FollowPath.clearRotationOverride() }))
    FollowPath.registerEventTrigger("startEject", superstructure.requestEjectCommand())
    FollowPath.registerEventTrigger("endEject", superstructure.requestIdleCommand())
  }

  val waitTime: Time
    get() = waitBeforeCommandSlider.get().seconds

  fun getCommand(superstructure: Superstructure, intake: Intake): Command {
    val mode = autonomousModeChooser.get()

    autoSelectionAlert.set(mode == null)
    autoSideAlert.set(fieldSideChooser.get() == null)

    val command =
        when (mode) {
          AutonomousMode.EXAMPLE_AUTO -> ExamplePathAuto(drivetrain, pathBuilder)
          AutonomousMode.WHEEL_RADIUS ->
              DriveCharacterizationCommands.wheelRadiusCharacterization(drivetrain)
          AutonomousMode.DRIVE_FF ->
              DriveCharacterizationCommands.feedforwardCharacterization(drivetrain)
          AutonomousMode.AUTOPOS -> TuningAutoPos(drivetrain, pathBuilder)
          AutonomousMode.INTAKE_QUAD_L1 ->
              IntakeQuadrantL1(drivetrain, superstructure, intake, pathBuilder).finallyDo { _ ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.INTAKE_QUAD_STEAL ->
              IntakeQuadrantSteal(drivetrain, superstructure, intake, pathBuilder).finallyDo { _ ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.PRELOAD_BUMP_CENTER ->
              PreloadL1Auto(drivetrain, superstructure, intake, pathBuilder).finallyDo { _ ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.INTAKE_FOLLOW_CLOSE ->
              IntakeQuadrantFollowClose(drivetrain, superstructure, intake, pathBuilder)
                  .finallyDo { _ -> FollowPath.clearRotationOverride() }
          AutonomousMode.INTAKE_FOLLOW_FAR ->
              IntakeQuadrantFollowFar(drivetrain, superstructure, intake, pathBuilder).finallyDo { _
                ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.EJECT_SWIPE ->
              EjectSwipe(drivetrain, superstructure, intake, pathBuilder).finallyDo { _ ->
                FollowPath.clearRotationOverride()
              }
          AutonomousMode.DO_NOTHING -> InstantCommand()
          else -> InstantCommand()
        }

    // note(nathan): saves one loop
    return if (waitTime == 0.seconds) command else WaitCommand(waitTime.inSeconds).andThen(command)
  }
}

private enum class AutonomousMode {
  EXAMPLE_AUTO,
  WHEEL_RADIUS,
  DRIVE_FF,
  AUTOPOS,
  INTAKE_QUAD_L1,
  INTAKE_QUAD_STEAL,
  PRELOAD_BUMP_CENTER,
  INTAKE_FOLLOW_CLOSE,
  INTAKE_FOLLOW_FAR,
  EJECT_SWIPE,
  DO_NOTHING,
}

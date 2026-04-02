package com.team4099.robot2026

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2026.auto.AutonomousSelector
import com.team4099.robot2026.commands.drivetrain.AimOTFCommand
import com.team4099.robot2026.commands.drivetrain.DrivePathOTF
import com.team4099.robot2026.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2026.commands.drivetrain.TargetAngleCommand
import com.team4099.robot2026.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.drivetrain.GyroIOPigeon2
import com.team4099.robot2026.subsystems.drivetrain.GyroIOSim
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOTalonFXReal
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOTalonFXSim
import com.team4099.robot2026.subsystems.leds.LedIO
import com.team4099.robot2026.subsystems.leds.LedIOCandle
import com.team4099.robot2026.subsystems.leds.Leds
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.climb.Climb
import com.team4099.robot2026.subsystems.superstructure.climb.ClimbIO
import com.team4099.robot2026.subsystems.superstructure.climb.ClimbIOSim
import com.team4099.robot2026.subsystems.superstructure.feeder.Feeder
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIO
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIOSim
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIOTalonFX
import com.team4099.robot2026.subsystems.superstructure.hopper.Hopper
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIO
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIOSim
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIOTalon
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIO
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIOSim
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIOTalon
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollers
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIO
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIOSim
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIOTalon
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIO
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIOSim
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIOTalon
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.subsystems.vision.camera.CameraIOPVSim
import com.team4099.robot2026.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2026.util.AllianceFlipUtil
import com.team4099.robot2026.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.defer
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.max
import org.team4099.lib.units.min

object RobotContainer {
  private val drivetrain: Drive
  private val vision: Vision
  private val climb: Climb
  private val feeder: Feeder
  private val hopper: Hopper
  val intake: Intake
  private val intakeRollers: IntakeRollers
  private val shooter: Shooter
  val superstructure: Superstructure
  val leds: Leds

  var driveSimulation: SwerveDriveSimulation? = null
  var isAligning = false

  var intakeOverridingAngle = IntakeConstants.ANGLES.INTAKE_ANGLE

  init {
    SimulatedArena.overrideInstance(Arena2026Rebuilt(false))

    if (Constants.Universal.DISABLE_COLLISIONS)
        SimulatedArena.overrideInstance(FieldConstants.EMPTY_MAPLESIM_FIELD)

    if (RobotBase.isReal()) {
      drivetrain =
          Drive(
              GyroIOPigeon2,
              ModuleIOTalonFXReal.generateModules(),
              { edu.wpi.first.math.geometry.Pose2d.kZero },
              { pose -> {} })
      vision =
          Vision(
              *VisionConstants.CAMERAS.map {
                    CameraIOPhotonvision(
                        it.value.first,
                        it.key,
                        it.value.second,
                        drivetrain::addVisionMeasurement,
                        { drivetrain.rotation },
                        { drivetrain.chassisSpeeds })
                  }
                  .toTypedArray(),
              poseSupplier = { drivetrain.pose })

      when (Constants.Universal.whoami) {
        Constants.WHOAMI.COMPBOT,
        Constants.WHOAMI.ALPHABOT -> {
          climb = Climb(object : ClimbIO {})
          feeder = Feeder(FeederIOTalonFX)
          hopper = Hopper(HopperIOTalon)
          intake = Intake(IntakeIOTalon)
          intakeRollers = IntakeRollers(IntakeRollersIOTalon)
          shooter = Shooter(ShooterIOTalon)
          leds =
              Leds(
                  { isAligning },
                  { Superstructure.Companion.SuperstructureStates.UNINITALIZED },
                  LedIOCandle(Constants.LEDS.CANDLE_ID))
        }
        Constants.WHOAMI.TESTBOT -> {
          climb = Climb(object : ClimbIO {})
          feeder = Feeder(object : FeederIO {})
          hopper = Hopper(object : HopperIO {})
          intake = Intake(object : IntakeIO {})
          intakeRollers = IntakeRollers(object : IntakeRollersIO {})
          shooter = Shooter(object : ShooterIO {})

          leds =
              Leds(
                  { isAligning },
                  { Superstructure.Companion.SuperstructureStates.UNINITALIZED },
                  object : LedIO {})
        }
      }
    } else {
      driveSimulation =
          SwerveDriveSimulation(Drive.mapleSimConfig, DrivetrainConstants.INITIAL_SIM_POSE)
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

      drivetrain =
          Drive(
              GyroIOSim(driveSimulation!!.gyroSimulation),
              ModuleIOTalonFXSim.generateModules(driveSimulation!!),
              driveSimulation!!::getSimulatedDriveTrainPose,
              driveSimulation!!::setSimulationWorldPose)

      vision =
          if (Constants.Universal.SIMULATE_VISION)
              Vision(
                  *VisionConstants.CAMERAS.map {
                        CameraIOPVSim(
                            it.value.first,
                            it.key,
                            it.value.second,
                            drivetrain::addVisionMeasurement,
                            { drivetrain.rotation },
                            { drivetrain.chassisSpeeds })
                      }
                      .toTypedArray(),
                  poseSupplier = { drivetrain.pose })
          else Vision(poseSupplier = { Pose2d() })

      climb = Climb(ClimbIOSim)
      feeder = Feeder(FeederIOSim)
      hopper = Hopper(HopperIOSim)
      intake = Intake(IntakeIOSim)
      intakeRollers = IntakeRollers(IntakeRollersIOSim)
      shooter = Shooter(ShooterIOSim)

      leds =
          Leds(
              { isAligning },
              { Superstructure.Companion.SuperstructureStates.UNINITALIZED },
              object : LedIO {})
    }

    superstructure =
        Superstructure(drivetrain, vision, climb, feeder, hopper, intake, intakeRollers, shooter)

    leds.stateSupplier = { superstructure.currentState }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
        TeleopDriveCommand(
            driver = Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain)
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.pose = Pose2d(drivetrain.pose.x, drivetrain.pose.y, 0.radians)
  }

  fun setDriveBrakeMode(neutralModeValue: NeutralModeValue = NeutralModeValue.Brake) {
    drivetrain.moduleIOs.forEach { it.toggleBrakeMode(neutralModeValue) }
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
    ControlBoard.forceHome.onTrue(superstructure.requestForceHomeCommand())
    ControlBoard.unjam.onTrue(superstructure.requestUnjamCommand())

    ControlBoard.forceIdle.onTrue(superstructure.requestIdleCommand())

    ControlBoard.prepScore.onTrue(superstructure.requestPrepScoreCommand())
    ControlBoard.score.onTrue(superstructure.requestScoreCommand())

    ControlBoard.score.onFalse(
        ConditionalCommand(superstructure.requestIdleCommand(), InstantCommand()) {
          superstructure.currentState == Superstructure.Companion.SuperstructureStates.PREP_SCORE ||
              superstructure.currentState == Superstructure.Companion.SuperstructureStates.SCORE ||
              superstructure.currentState ==
                  Superstructure.Companion.SuperstructureStates.SCORE_AND_INTAKE
        })
    ControlBoard.manualScore.onTrue(
        Commands.defer(
            {
              Commands.runOnce({
                superstructure.overrideShooterVelocity = !superstructure.overrideShooterVelocity
              })
            },
            setOf(superstructure)))
    ControlBoard.defenseMode.onTrue(
        Commands.defer(
            { Commands.runOnce({ superstructure.defenseMode = !superstructure.defenseMode }) },
            setOf(superstructure)))

    //    ControlBoard.prepClimb.onTrue(superstructure.requestPrepClimbCommand())
    //    ControlBoard.climb.onTrue(superstructure.requestClimbCommand())

    ControlBoard.intake.onTrue(superstructure.requestIntakeCommand())
    ControlBoard.intake.onFalse(superstructure.requestIdleCommand())

    ControlBoard.forceIntakeFullUp.whileTrue(
        RepeatCommand(
            SequentialCommandGroup(
                Commands.runOnce({
                  intakeOverridingAngle =
                      min(IntakeConstants.PIVOT_MAX_ANGLE, intakeOverridingAngle + 20.degrees)
                }),
                Commands.defer(
                    { superstructure.requestForceIntakeCommand(intakeOverridingAngle) },
                    setOf(superstructure)),
                WaitCommand(0.1))))
    ControlBoard.forceIntakeFullDown.whileTrue(
        RepeatCommand(
            SequentialCommandGroup(
                Commands.runOnce({
                  intakeOverridingAngle =
                      max(IntakeConstants.PIVOT_MIN_ANGLE, intakeOverridingAngle - 20.degrees)
                }),
                Commands.defer(
                    { superstructure.requestForceIntakeCommand(intakeOverridingAngle) },
                    setOf(superstructure)),
                WaitCommand(0.1))))

    ControlBoard.jiggle.whileTrue(
        RepeatCommand(
            SequentialCommandGroup(
                defer(
                    {
                      superstructure.requestForceIntakeCommand(
                          max(IntakeConstants.PIVOT_MIN_ANGLE, intakeOverridingAngle - 5.degrees))
                    },
                    setOf()),
                WaitCommand(0.2),
                defer(
                    {
                      superstructure.requestForceIntakeCommand(
                          min(IntakeConstants.PIVOT_MAX_ANGLE, intakeOverridingAngle + 5.degrees))
                    },
                    setOf()),
                WaitCommand(0.2),
            )))

    ControlBoard.rotateTrench.whileTrue(
        TargetAngleCommand(
            Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain,
            {
              if (FieldConstants.inTrenchAllianceZone(drivetrain.pose) &&
                  !AllianceFlipUtil.shouldFlip() ||
                  !FieldConstants.inTrenchAllianceZone(drivetrain.pose) &&
                      AllianceFlipUtil.shouldFlip())
                  0.degrees
              else 180.degrees
            }))
    ControlBoard.rotateBump.whileTrue(
        TargetAngleCommand(
            Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain,
            {
              if (FieldConstants.inTrenchAllianceZone(drivetrain.pose) &&
                  !AllianceFlipUtil.shouldFlip() ||
                  !FieldConstants.inTrenchAllianceZone(drivetrain.pose) &&
                      AllianceFlipUtil.shouldFlip())
                  45.degrees
              else 225.degrees
            }))

    ControlBoard.score.whileTrue(
        ConditionalCommand(
            AimOTFCommand(
                drivetrain,
                { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
                { ControlBoard.slowMode },
                driver = Jessika(),
            ),
            InstantCommand()) {
              !superstructure.overrideShooterVelocity
              //              superstructure.currentState ==
              // Superstructure.Companion.SuperstructureStates.SCORE ||
              //                  superstructure.currentState ==
              //                      Superstructure.Companion.SuperstructureStates.PREP_SCORE
            })

    ControlBoard.leftTrenchOTF.whileTrue(
        ConditionalCommand(
            DrivePathOTF.allianceZoneToNeutralInLeftTrench(drivetrain),
            DrivePathOTF.neutralZoneToAllianceInLeftTrench(drivetrain)) {
              FieldConstants.inTrenchAllianceZone(drivetrain.pose)
            })
    ControlBoard.rightTrenchOTF.whileTrue(
        ConditionalCommand(
            DrivePathOTF.allianceZoneToNeutralInRightTrench(drivetrain),
            DrivePathOTF.neutralZoneToAllianceInRightTrench(drivetrain)) {
              FieldConstants.inTrenchAllianceZone(drivetrain.pose)
            })

    ControlBoard.climbOTF.whileTrue(
        ConditionalCommand(
            DrivePathOTF.alignClimbBottom(drivetrain), DrivePathOTF.alignClimbTop(drivetrain)) {
              FieldConstants.inClimbLowerHalf(drivetrain.pose)
            })

    ControlBoard.eject.onTrue(superstructure.requestEjectCommand())
  }

  fun mapTestControls() {}

  fun mapTunableCommands() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain, vision, superstructure)

  fun resetSimulationField() {
    if (!RobotBase.isSimulation()) return

    driveSimulation!!.setSimulationWorldPose(DrivetrainConstants.INITIAL_SIM_POSE)
    SimulatedArena.getInstance().resetFieldForAuto()
  }

  fun updateSimulation() {
    if (!RobotBase.isSimulation()) return

    SimulatedArena.getInstance().simulationPeriodic()
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose)
    Logger.recordOutput(
        "FieldSimulation/Fuel", *SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"))
  }
}

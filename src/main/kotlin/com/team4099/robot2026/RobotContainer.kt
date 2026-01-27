package com.team4099.robot2026

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2026.auto.AutonomousSelector
import com.team4099.robot2026.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2026.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.drivetrain.GyroIOPigeon2
import com.team4099.robot2026.subsystems.drivetrain.GyroIOSim
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOTalonFXReal
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOTalonFXSim
import com.team4099.robot2026.subsystems.superstructure.Superstructure
import com.team4099.robot2026.subsystems.superstructure.climb.Climb
import com.team4099.robot2026.subsystems.superstructure.climb.ClimbIOSim
import com.team4099.robot2026.subsystems.superstructure.climb.ClimbIOTalon
import com.team4099.robot2026.subsystems.superstructure.feeder.Feeder
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIOSim
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIOTalonFX
import com.team4099.robot2026.subsystems.superstructure.hopper.Hopper
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIOSim
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIOTalon
import com.team4099.robot2026.subsystems.superstructure.intake.Intake
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIOSim
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIOTalon
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollers
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIOSim
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIOTalon
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIOSim
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIOTalon
import com.team4099.robot2026.subsystems.vision.Vision
import com.team4099.robot2026.subsystems.vision.camera.CameraIO
import com.team4099.robot2026.subsystems.vision.camera.CameraIOPVSim
import com.team4099.robot2026.subsystems.vision.camera.CameraIOPhotonvision
import com.team4099.robot2026.util.driver.Jessika
import edu.wpi.first.wpilibj.RobotBase
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Transform3d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians

object RobotContainer {
  private val drivetrain: Drive
  private val vision: Vision
  private val climb: Climb
  private val feeder: Feeder
  private val hopper: Hopper
  private val intake: Intake
  private val intakeRollers: IntakeRollers
  private val shooter: Shooter
  val superstructure: Superstructure

  var driveSimulation: SwerveDriveSimulation? = null

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
              CameraIOPhotonvision(
                  CameraIO.DetectionPipeline.APRIL_TAG,
                  VisionConstants.CAMERA_NAMES[0],
                  VisionConstants.CAMERA_TRANSFORMS[0],
                  drivetrain::addVisionMeasurement,
                  { drivetrain.rotation }),
              CameraIOPhotonvision(
                  CameraIO.DetectionPipeline.APRIL_TAG,
                  VisionConstants.CAMERA_NAMES[1],
                  VisionConstants.CAMERA_TRANSFORMS[1],
                  drivetrain::addVisionMeasurement,
                  { drivetrain.rotation }),
              poseSupplier = { drivetrain.pose })

      climb = Climb(ClimbIOTalon)
      feeder = Feeder(FeederIOTalonFX)
      hopper = Hopper(HopperIOTalon)
      intake = Intake(IntakeIOTalon)
      intakeRollers = IntakeRollers(IntakeRollersIOTalon)
      shooter = Shooter(ShooterIOTalon)
    } else {
      driveSimulation =
          SwerveDriveSimulation(
              Drive.mapleSimConfig, DrivetrainConstants.INITIAL_SIM_POSE.toPose2d())
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

      drivetrain =
          Drive(
              GyroIOSim(driveSimulation!!.gyroSimulation),
              ModuleIOTalonFXSim.generateModules(driveSimulation!!),
              driveSimulation!!::getSimulatedDriveTrainPose,
              driveSimulation!!::setSimulationWorldPose)

      if (Constants.Universal.SIMULATE_VISION)
          vision =
              Vision(
                  CameraIOPVSim(
                      CameraIO.DetectionPipeline.OBJECT_DETECTION,
                      "raven1",
                      Transform3d(),
                      drivetrain::addVisionMeasurement,
                      { drivetrain.rotation }),
                  poseSupplier = { drivetrain.pose })
      else vision = Vision(poseSupplier = { Pose3d() })

      climb = Climb(ClimbIOSim)
      feeder = Feeder(FeederIOSim)
      hopper = Hopper(HopperIOSim)
      intake = Intake(IntakeIOSim)
      intakeRollers = IntakeRollers(IntakeRollersIOSim)
      shooter = Shooter(ShooterIOSim)
    }

    superstructure =
        Superstructure(drivetrain, vision, climb, feeder, hopper, intake, intakeRollers, shooter)
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
        TeleopDriveCommand(
            driver = Jessika(),
            { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
            { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
            { ControlBoard.slowMode },
            drivetrain,
        )
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.pose = Pose3d(drivetrain.pose.x, drivetrain.pose.y, drivetrain.pose.z, Rotation3d())
  }

  fun setDriveBrakeMode(neutralModeValue: NeutralModeValue = NeutralModeValue.Brake) {
    //    drivetrain.configNeutralMode(neutralModeValue)
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))

    ControlBoard.forceIdle.onTrue(superstructure.requestIdleCommand())

    ControlBoard.prepScore.onTrue(superstructure.requestPrepScoreCommand())
    ControlBoard.score.onTrue(superstructure.requestScoreCommand())

    ControlBoard.prepClimb.onTrue(superstructure.requestPrepClimbCommand())
    ControlBoard.climb.onTrue(superstructure.requestClimbCommand())

    ControlBoard.intake.onTrue(superstructure.requestIntakeCommand())
    ControlBoard.forceIntakeUp.onTrue(superstructure.requestForceIntakeUpCommand())
    ControlBoard.forceIntakeDown.onTrue(superstructure.requestForceIntakeDownCommand())

    ControlBoard.eject.onTrue(superstructure.requestEjectCommand())
  }

  fun mapTestControls() {}

  fun mapTunableCommands() {}

  fun getAutonomousCommand() = AutonomousSelector.getCommand(drivetrain, vision)

  fun resetSimulationField() {
    if (!RobotBase.isSimulation()) return

    driveSimulation!!.setSimulationWorldPose(Pose2d(3.meters, 3.meters, 0.radians).pose2d)
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

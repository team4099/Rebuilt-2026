package com.team4099.robot2026

import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.subsystems.superstructure.Request
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.wpilib.hardware.hal.HAL
import org.wpilib.hardware.hal.RobotMode
import org.wpilib.simulation.DriverStationSim
import org.wpilib.driverstation.internal.DriverStationBackend

class SimRobotControlsTest {
  @Test
  fun `teleop inputs schedule commands and move robot`() {
    System.setProperty("robot", "compbot")
    System.setProperty("tuning", "f")
    HAL.initialize()
    val robot = Robot
    DriverStationBackend.observeUserProgramStarting()
    DriverStationSim.setJoystickAxesMaximumIndex(0, 6)
    DriverStationSim.setJoystickButtonsMaximumIndex(0, 12)
    DriverStationSim.setJoystickIsGamepad(0, true)
    val macSimulation = System.getProperty("os.name").startsWith("Mac")
    val turnAxis = if (macSimulation) 2 else 4
    val intakeAxis = if (macSimulation) 4 else 2
    val scoreAxis = if (macSimulation) 5 else 3
    DriverStationSim.setDsAttached(true)
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED)
    DriverStationSim.setEnabled(true)
    DriverStationSim.setJoystickAxis(0, 1, -0.8)
    DriverStationSim.notifyNewData()
    robot.teleopInit()
    val field = RobotContainer::class.java.getDeclaredField("drivetrain")
    field.isAccessible = true
    val drive = field.get(null) as Drive
    val start = drive.pose.pose2d.translation
    repeat(100) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    val distance = drive.pose.pose2d.translation.getDistance(start)
    assertTrue(distance > 0.5, "Expected motion from default command, got $distance m")
    DriverStationSim.setJoystickAxis(0, 1, 0.0)
    DriverStationSim.setJoystickAxis(0, 0, -0.8)
    val beforeStrafe = drive.pose.pose2d.translation
    repeat(80) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(drive.pose.pose2d.translation.getDistance(beforeStrafe) > 0.3, "Strafe: from $beforeStrafe to ${drive.pose.pose2d.translation}, input ${com.team4099.robot2026.config.ControlBoard.strafe}, speed ${drive.chassisSpeeds.vy.value}")
    DriverStationSim.setJoystickAxis(0, 0, 0.0)
    DriverStationSim.setJoystickAxis(0, turnAxis, 0.8)
    val beforeTurn = drive.pose.pose2d.rotation.radians
    repeat(80) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(kotlin.math.abs(drive.pose.pose2d.rotation.radians - beforeTurn) > 0.1, "Turn did not rotate robot")
    assertTrue(!com.team4099.robot2026.config.ControlBoard.intake.asBoolean, "Turning activated intake")
    DriverStationSim.setJoystickAxis(0, turnAxis, 0.0)
    DriverStationSim.setJoystickAxis(0, intakeAxis, 1.0)
    repeat(100) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(RobotContainer.superstructure.currentRequest is Request.SuperstructureRequest.Intake, "Intake trigger did not request intake: ${RobotContainer.superstructure.currentRequest}")
    assertTrue(kotlin.math.abs(com.team4099.robot2026.config.ControlBoard.turn) < 0.01, "Intake trigger activated turn")
    DriverStationSim.setJoystickAxis(0, intakeAxis, 0.0)
    DriverStationSim.setJoystickAxis(0, scoreAxis, 1.0)
    repeat(100) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(RobotContainer.superstructure.currentRequest is Request.SuperstructureRequest.Score, "Score trigger did not request scoring")
    val shooterField = RobotContainer::class.java.getDeclaredField("shooter")
    shooterField.isAccessible = true
    val shooter = shooterField.get(null) as com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
    assertTrue(shooter.inputs.shooterLeaderVelocity.value > 1.0, "Shooter did not spin up")
    val nt = org.wpilib.networktables.NetworkTableInstance.getDefault()
    assertTrue(nt.getTopic("/AdvantageKit/Drive/Module0/driveVelocityDegPerSec").exists(), "Drivetrain input not published")
    assertTrue(nt.getTopic("/AdvantageKit/RealOutputs/Superstructure/currentState").exists(), "Superstructure state not published")
    DriverStationSim.setEnabled(false)
    DriverStationSim.setJoystickAxis(0, scoreAxis, 0.0)
    DriverStationSim.notifyNewData()
    nt.getEntry("/SmartDashboard/AutonomousMode/selected/tune").setString("Auto Pose Tuner DO NOT RUN AT COMPETITION")
    nt.getEntry("/SmartDashboard/FieldSide/selected/tune").setString("Left")
    repeat(30) {
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.disabledPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(10)
    }
    assertTrue(robot.autonomousCommand.javaClass.simpleName == "TuningAutoPos", "Autonomous chooser did not select the requested mode: ${robot.autonomousCommand.javaClass.simpleName}")
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS)
    DriverStationSim.setEnabled(true)
    DriverStationSim.notifyNewData()
    robot.autonomousInit()
    val autoStart = drive.pose.pose2d.translation
    repeat(120) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(drive.pose.pose2d.translation.getDistance(autoStart) > 0.2, "Autonomous command did not move robot")
    DriverStationSim.setEnabled(false)
    DriverStationSim.notifyNewData()
    for (selection in listOf(
        "Example Auto DO NOT RUN AT COMPETITION",
        "Intake Quadrant L1",
        "Intake Quadrant Steal",
        "Preload + Bump Center",
        "Intake Follow Close (ADD A WAIT TIME)",
        "Intake Follow Far (ADD A WAIT TIME)",
        "Intake Eject Full Swipe Shoot")) {
      nt.getEntry("/SmartDashboard/AutonomousMode/selected/tune").setString(selection)
      repeat(8) {
        org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
        robot.disabledPeriodic()
        org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
        Thread.sleep(10)
      }
      val active = nt.getEntry("/SmartDashboard/AutonomousMode/selected/value").getString("")
      assertTrue(active == selection, "Chooser selected $active instead of $selection")
      assertTrue(robot.autonomousCommand.javaClass.simpleName != "InstantCommand", "Auto $selection did not load")
      DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS)
      DriverStationSim.setEnabled(true)
      DriverStationSim.notifyNewData()
      robot.autonomousInit()
      repeat(20) {
        DriverStationSim.notifyNewData()
        org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
        robot.robotPeriodic()
        org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
        Thread.sleep(5)
      }
      assertTrue(drive.pose.pose2d.x.isFinite() && drive.pose.pose2d.y.isFinite(), "Auto $selection produced an invalid pose")
      DriverStationSim.setEnabled(false)
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
    }
    nt.getEntry("/SmartDashboard/AutonomousMode/selected/tune").setString("Do nothing")
    nt.getEntry("/Shuffleboard/Pre-match/Wait Time").setDouble(0.3)
    repeat(30) {
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      org.wpilib.tunable.TunableRegistry.update()
      robot.disabledPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(10)
    }
    assertTrue(kotlin.math.abs(RobotContainer.autonomousSelector.waitTime.value - 0.3) < 0.001, "Wait time was not received")
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS)
    DriverStationSim.setEnabled(true)
    DriverStationSim.notifyNewData()
    robot.autonomousInit()
    assertTrue(robot.autonomousCommand.isScheduled, "Delayed auto did not schedule")
    repeat(10) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(robot.autonomousCommand.isScheduled, "Auto ended before the requested wait")
    repeat(100) {
      DriverStationSim.notifyNewData()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicBeforeUser()
      robot.robotPeriodic()
      org.littletonrobotics.junction.Logger.AdvancedHooks.invokePeriodicAfterUser(0, 0)
      Thread.sleep(5)
    }
    assertTrue(!robot.autonomousCommand.isScheduled, "Auto did not finish after the wait")
    DriverStationSim.setEnabled(false)
    DriverStationSim.notifyNewData()
    org.littletonrobotics.junction.Logger.end()
  }
}

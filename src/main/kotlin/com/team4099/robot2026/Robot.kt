package com.team4099.robot2026

import bline.commands.FollowPathV2 as FollowPath
import com.ctre.phoenix6.SignalLogger
import com.team4099.lib.hal.Clock
import com.team4099.robot2026.commands.drivetrain.DrivePathOTF
import com.team4099.robot2026.config.ControlBoard
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.IntakeConstants
import com.team4099.robot2026.config.constants.RollersConstants
import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.NTSafePublisher
import java.nio.file.Files
import java.nio.file.Paths
import org.ejml.EjmlVersion.BUILD_DATE
import org.ejml.EjmlVersion.DIRTY
import org.ejml.EjmlVersion.GIT_BRANCH
import org.ejml.EjmlVersion.GIT_SHA
import org.ejml.EjmlVersion.MAVEN_NAME
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedPowerDistribution
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.seconds
import org.wpilib.command2.Command
import org.wpilib.command2.CommandScheduler
import org.wpilib.driverstation.Alliance
import org.wpilib.driverstation.MatchState
import org.wpilib.framework.RobotBase
import org.wpilib.hardware.hal.AllianceStationID
import org.wpilib.hardware.power.PowerDistribution
import org.wpilib.net.WebServer
import org.wpilib.simulation.DriverStationSim
import org.wpilib.system.Filesystem
import org.wpilib.util.Alert
import org.wpilib.util.Alert.Level as AlertType

object Robot : LoggedRobot() {
  val logFolderAlert =
      Alert(
          "logFolderMissing",
          "Log folder path does not exist. Data will NOT be logged.",
          AlertType.HIGH)
  val logReceiverQueueAlert =
      Alert(
          "logQueueOverflow",
          "Logging queue exceeded capacity, data will NOT be logged.",
          AlertType.HIGH)
  val logOpenFileAlert =
      Alert("logFileOpenFailed", "Failed to open log file. Data will NOT be logged", AlertType.HIGH)
  val logWriteAlert =
      Alert(
          "logFileWriteFailed",
          "Failed write to the log file. Data will NOT be logged",
          AlertType.HIGH)
  val logSimulationAlert = Alert("simulation", "Running in simulation", AlertType.LOW)
  val logTuningModeEnabled =
      Alert("tuningMode", "Tuning Mode Enabled. Expect loop times to be greater", AlertType.MEDIUM)

  lateinit var autonomousCommand: Command
  var scoringFirst: Boolean = false
  var autoStartTime: Time = (-1337).seconds

  init {
    // elastic layout upload dont remove
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath())
    // running replays as fast as possible when replaying. (play in real time when robot is real or
    // sim)
    setUseTiming(
        RobotBase.isReal() || Constants.Universal.SIM_MODE != Constants.Tuning.SimType.REPLAY)

    // Tuning mode alert check
    logTuningModeEnabled.set(Constants.Tuning.TUNING_MODE)

    // metadata value (not timed -- just metadata for given log file)
    Logger.recordMetadata(
        "Robot", if (RobotBase.isReal()) "REAL" else Constants.Universal.SIM_MODE.name)
    Logger.recordMetadata("Tuning Mode Enabled", Constants.Tuning.TUNING_MODE.toString())
    Logger.recordMetadata("ProjectName", MAVEN_NAME)
    Logger.recordMetadata("BuildDate", BUILD_DATE)
    Logger.recordMetadata("GitSHA", GIT_SHA)
    Logger.recordMetadata("GitBranch", GIT_BRANCH)
    when (DIRTY) {
      0 -> Logger.recordMetadata("GitDirty", "All changes committed")
      1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes")
      else -> Logger.recordMetadata("GitDirty", "Unknown")
    }

    var isLogging = false
    if (RobotBase.isReal()) {
      // check if folder path exists
      if (Files.exists(Paths.get(Constants.Universal.LOG_FOLDER))) {
        // log to USB stick and network for real time data viewing on AdvantageScope
        isLogging = true
        Logger.addDataReceiver(WPILOGWriter(Constants.Universal.LOG_FOLDER))
      } else {
        logFolderAlert.set(true)
      }

      Logger.addDataReceiver(NTSafePublisher())
      LoggedPowerDistribution.getInstance(
          Constants.Universal.POWER_DISTRIBUTION_HUB_ID, 0, PowerDistribution.ModuleType.REV)
    } else {
      when (Constants.Universal.SIM_MODE) {
        Constants.Tuning.SimType.SIM -> {
          isLogging = true
          Logger.addDataReceiver(NTSafePublisher())
          Logger.addDataReceiver(WPILOGWriter()) // writes to ./logs
          logSimulationAlert.set(true)
          DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1)
        }
        Constants.Tuning.SimType.REPLAY -> {
          // if in replay mode get file path from command line and read log file
          val path = LogFileUtil.findReplayLog()
          Logger.setReplaySource(WPILOGReader(path))
          Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")))
        }
      }

      // initialize mech2d stuff
    }

    Logger.start() // no more configuration allowed

    Logger.recordOutput("LogFolder/isLogging", isLogging)

    // change whats commented below if using hoot logs
    SignalLogger.enableAutoLogging(false)
    SignalLogger.stop()
    //    SignalLogger.setPath("/media/sda1/ctre-logs/")

    // init a buncha things
    FieldConstants.fieldLayout
    RobotContainer.mapControls()
    CommandScheduler.getInstance().schedule(DrivePathOTF.warmupCommand())

    // Set the scheduler to log events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize { command: Command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", true)
    }

    CommandScheduler.getInstance().onCommandFinish { command: Command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", false)
    }

    CommandScheduler.getInstance().onCommandInterrupt { command: Command ->
      Logger.recordOutput("/ActiveCommands/${command.name}", false)
    }

    Logger.recordOutput("TuningMode", Constants.Tuning.TUNING_MODE)

    FollowPath.setDoubleLoggingConsumer { value ->
      CustomLogger.recordOutput(value.getFirst(), value.getSecond())
    }
    FollowPath.setBooleanLoggingConsumer { value ->
      CustomLogger.recordOutput(value.getFirst(), value.getSecond())
    }
    FollowPath.setPoseLoggingConsumer { value ->
      CustomLogger.recordOutput(value.getFirst(), value.getSecond())
    }
    FollowPath.setTranslationListLoggingConsumer { value ->
      CustomLogger.recordOutput(value.getFirst(), value.getSecond())
    }
  }

  override fun autonomousInit() {
    CommandScheduler.getInstance().schedule(autonomousCommand)
    RobotContainer.intake.setBrakeMode(true)
    RobotContainer.intake.setSupplyCurrentLimit(IntakeConstants.AUTO_SUPPLY_CURRENT_LIMIT)
    RobotContainer.intakeRollers.setSupplyCurrentLimit(RollersConstants.AUTO_SUPPLY_CURRENT_LIMIT)
    autoStartTime = Clock.timestamp
  }

  override fun disabledPeriodic() {
    autoStartTime = -1337.seconds
    autonomousCommand = RobotContainer.getAutonomousCommand()
  }

  override fun disabledInit() {
    RobotContainer.resetSimulationField()
    RobotContainer.intake.setBrakeMode(false)
  }

  override fun robotPeriodic() {
    val startTime = Clock.timestamp

    // begin scheduling all commands
    CommandScheduler.getInstance().run()

    // checking for logging errors
    logReceiverQueueAlert.set(Logger.getReceiverQueueFault())

    Logger.recordOutput(
        "LoggedRobot/RemainingRamMB", Runtime.getRuntime().freeMemory() / 1024 / 1024)

    ControlBoard.driverRumbleConsumer.accept(false)
    ControlBoard.operatorRumbleConsumer.accept(false)

    CustomLogger.recordDebugOutput(
        "LoggedRobot/totalMS", (Clock.timestamp - startTime).inMilliseconds)
  }

  override fun teleopInit() {
    RobotContainer.getAutonomousCommand().cancel()
    scoringFirst =
        (MatchState.getGameData().orElse("") == "R" &&
            (MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.BLUE)) ||
            (MatchState.getGameData().orElse("") == "B" &&
                (MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED))
    //    if (RobotContainer.superstructure.currentState ==
    //        Superstructure.Companion.SuperstructureStates.CLIMB) {
    //     // RobotContainer.superstructure.currentRequest =
    // Request.SuperstructureRequest.ExtendClimb()
    //    } else {
    RobotContainer.superstructure.currentRequest = Request.SuperstructureRequest.Idle()
    //   }
    RobotContainer.intake.setBrakeMode(true)
    RobotContainer.intake.setSupplyCurrentLimit(IntakeConstants.TELEOP_SUPPLY_CURRENT_LIMIT)
    RobotContainer.intakeRollers.setSupplyCurrentLimit(RollersConstants.SUPPLY_CURRENT_LIMIT)
  }

  override fun teleopPeriodic() {
    val time = MatchState.getMatchTime()
    val shiftIndex =
        when {
          time > 125 -> 0
          time > 100 -> 1
          time > 75 -> 2
          time > 50 -> 3
          time > 25 -> 4
          else -> 5
        }
    if (scoringFirst) {
      CustomLogger.recordOutput("MatchData/ScoringNow", shiftIndex % 2 == 0 || shiftIndex == 5)
    } else {
      CustomLogger.recordOutput("MatchData/ScoringNow", shiftIndex % 2 != 0)
    }
    if (125 > time && time > 25) {
      CustomLogger.recordOutput("MatchData/ShiftTimeLeft", time % 25)
    } else if (time > 125) {
      CustomLogger.recordOutput("MatchData/ShiftTimeLeft", time % 10)
    } else {
      CustomLogger.recordOutput("MatchData/ShiftTimeLeft", time % 30)
    }
  }

  override fun utilityInit() {
    RobotContainer.getAutonomousCommand().cancel()
    RobotContainer.intake.setBrakeMode(true)
  }
}

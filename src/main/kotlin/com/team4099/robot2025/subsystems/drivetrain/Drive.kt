// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package com.team4099.robot2025.subsystems.drivetrain

import com.ctre.phoenix6.CANBus
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.PathPlannerLogging
import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.util.LocalADStarAK
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterPerSecond
import org.team4099.lib.units.derived.inMetersPerSecondPerMeterSeconds
import org.team4099.lib.units.derived.inMetersPerSecondPerMetersPerSecond
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadian
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadianSeconds
import org.team4099.lib.units.derived.inRadiansPerSecondPerRadiansPerSecond
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.absoluteValue
import kotlin.math.hypot
import kotlin.math.max
import edu.wpi.first.math.geometry.Pose2d as WPIPose2d
import edu.wpi.first.math.geometry.Pose3d as WPIPose3d
import edu.wpi.first.math.kinematics.ChassisSpeeds as WPIChassisSpeeds

class Drive(
  private val gyroIO: GyroIO,
  moduleIOs: Array<ModuleIO>,
  val getSimulationPoseCallback: Supplier<edu.wpi.first.math.geometry.Pose2d>,
  val resetSimulationPoseCallback: Consumer<edu.wpi.first.math.geometry.Pose2d>
) : SubsystemBase() {
  private val gyroInputs: GyroIO.GyroIOInputs = GyroIO.GyroIOInputs()
  private val modules = arrayOfNulls<Module>(4) // FL, FR, BL, BR
  private val sysId: SysIdRoutine
  private val gyroDisconnectedAlert =
    Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError)

  private val kinematics: SwerveDriveKinematics =
    SwerveDriveKinematics(
      *(
        moduleTranslations
          .map { translation: Translation2d -> translation.translation2d }
          .toTypedArray()
        )
    )
  private var rawGyroRotation: Rotation3d = Rotation3d()
  private val lastModulePositions: Array<SwerveModulePosition?> = // For delta tracking
    arrayOf(
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition(),
      SwerveModulePosition()
    )
  private val poseEstimator: SwerveDrivePoseEstimator3d =
    SwerveDrivePoseEstimator3d(
      kinematics, rawGyroRotation.rotation3d, lastModulePositions, Pose3d().pose3d
    )

  init {
    modules[0] = Module(moduleIOs[0], 0, TunerConstants.FrontLeft)
    modules[1] = Module(moduleIOs[1], 1, TunerConstants.FrontRight)
    modules[2] = Module(moduleIOs[2], 2, TunerConstants.BackLeft)
    modules[3] = Module(moduleIOs[3], 3, TunerConstants.BackRight)

    // Usage reporting for swerve template
    HAL.report(
      FRCNetComm.tResourceType.kResourceType_RobotDrive,
      FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit
    )

    // Start odometry thread
    PhoenixOdometryThread.instance.start()

    // Configure AutoBuilder for PathPlanner
    // WARNING! takes in pose as pose2d and does not account for z
    AutoBuilder.configure(
      { this.pose.pose3d.toPose2d() },
      { pose: WPIPose2d -> this.pose = Pose3d(Pose2d(pose)) },
      { this.chassisSpeeds.chassisSpeedsWPILIB },
      { speeds: WPIChassisSpeeds -> this.runSpeeds(ChassisSpeeds(speeds)) },
      PPHolonomicDriveController(
        PIDConstants(
          DrivetrainConstants.PID.AUTO_POS_KD.inMetersPerSecondPerMetersPerSecond,
          DrivetrainConstants.PID.AUTO_POS_KI.inMetersPerSecondPerMeterSeconds,
          DrivetrainConstants.PID.AUTO_POS_KD.inMetersPerSecondPerMeterPerSecond
        ),
        PIDConstants(
          DrivetrainConstants.PID.AUTO_THETA_PID_KP.inRadiansPerSecondPerRadian,
          DrivetrainConstants.PID.AUTO_THETA_PID_KD.inRadiansPerSecondPerRadiansPerSecond,
          DrivetrainConstants.PID.AUTO_THETA_PID_KI.inRadiansPerSecondPerRadianSeconds
        )
      ),
      PP_CONFIG,
      {
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
          DriverStation.Alliance.Red
      },
      this
    )
    Pathfinding.setPathfinder(LocalADStarAK())
    PathPlannerLogging.setLogActivePathCallback { activePath: List<WPIPose2d> ->
      Logger.recordOutput("Odometry/Trajectory", *activePath.toTypedArray<WPIPose2d>())
    }
    PathPlannerLogging.setLogTargetPoseCallback { targetPose: WPIPose2d ->
      Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
    }

    // Configure SysId
    sysId =
      SysIdRoutine(
        SysIdRoutine.Config(Volts.of(1.0) / Seconds.of(1.0), Volts.of(7.0), Seconds.of(10.0)) {
          state: SysIdRoutineLog.State ->
          Logger.recordOutput("Drive/SysIdState", state.toString())
        },
        SysIdRoutine.Mechanism(
          { voltage: Voltage -> runCharacterization(voltage.`in`(Units.Volts)) }, null, this
        )
      )
  }

  override fun periodic() {
    val startTime = Clock.fpgaTime

    odometryLock.lock() // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs)
    Logger.processInputs("Drive/Gyro", gyroInputs)
    for (module in modules) {
      module!!.periodic()
    }
    odometryLock.unlock()

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (module in modules) {
        module!!.stop()
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
      Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
    }

    // Update odometry
    val sampleTimestamps = modules[0]!!.odometryTimestamps // All signals are sampled together
    val sampleCount = sampleTimestamps.size
    for (i in 0 until sampleCount) {
      // Read wheel positions and deltas from each module
      val modulePositions: Array<SwerveModulePosition?> = arrayOfNulls(4)
      val moduleDeltas: Array<SwerveModulePosition?> = arrayOfNulls(4)
      for (moduleIndex in 0..3) {
        modulePositions[moduleIndex] = modules[moduleIndex]!!.odometryPositions[i]
        moduleDeltas[moduleIndex] =
          SwerveModulePosition(
            modulePositions[moduleIndex]!!.distanceMeters -
              lastModulePositions[moduleIndex]!!.distanceMeters,
            modulePositions[moduleIndex]!!.angle
          )
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation =
          Rotation3d(
            gyroInputs.odometryRollPositions[i],
            gyroInputs.odometryPitchPositions[i],
            gyroInputs.odometryYawPositions[i]
          )
      } else {
        // Use the angle delta from the kinematics and module deltas
        val twist = Twist2d(kinematics.toTwist2d(*moduleDeltas))
        rawGyroRotation += Rotation3d(0.radians, 0.radians, twist.dtheta)
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation.rotation3d, modulePositions)
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && RobotBase.isReal())

    Logger.recordOutput("Odometry/pose", pose.pose3d)
    Logger.recordOutput("SwerveChassisSpeeds/Measured", chassisSpeeds.chassisSpeedsWPILIB)

    Logger.recordOutput("SwerveStates/Measured", *moduleStates)

    CustomLogger.recordOutput(
      "LoggedRobot/Subsystems/DriveLoopTimeMS", (Clock.fpgaTime - startTime).inMilliseconds
    )
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  fun runSpeeds(speeds: ChassisSpeeds, flipIfRed: Boolean = true) {
    val flippedSpeeds =
      if (flipIfRed && AllianceFlipUtil.shouldFlip())
        ChassisSpeeds(-speeds.vx, -speeds.vy, speeds.omega)
      else speeds
    // Calculate module setpoints
    val discreteSpeeds =
      ChassisSpeeds.discretize(flippedSpeeds, Constants.Universal.LOOP_PERIOD_TIME)
    val setpointStates: Array<SwerveModuleState> =
      kinematics.toSwerveModuleStates(discreteSpeeds.chassisSpeedsWPILIB)

    SwerveDriveKinematics.desaturateWheelSpeeds(
      setpointStates, TunerConstants.kSpeedAt12Volts.inMetersPerSecond
    )

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds.chassisSpeedsWPILIB)

    // Send setpoints to modules
    for (i in 0..3) {
      modules[i]!!.runSetpoint(setpointStates[i])
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
  }

  /** Point the module's wheels at the direction specified */
  fun pointWheelsAt(angle: Angle) {
    for (i in 0..3) {
      modules[i]!!.pointWheelsAt(angle)
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  fun runCharacterization(output: Double) {
    for (i in 0..3) {
      modules[i]!!.runCharacterization(output)
    }
  }

  /** Stops the drive. */
  fun stop() {
    runSpeeds(ChassisSpeeds())
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  fun stopWithX() {
    val headings: Array<Angle> = Array(4) { 0.radians }
    for (i in 0..3) {
      headings[i] = Angle(moduleTranslations[i].x.inMeters, moduleTranslations[i].y.inMeters)
    }
    kinematics.resetHeadings(
      *(headings.map { heading: Angle -> heading.inRotation2ds }.toTypedArray())
    )
    stop()
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
    return run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.quasistatic(direction))
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
    return run { runCharacterization(0.0) }.withTimeout(1.0).andThen(sysId.dynamic(direction))
  }

  private val moduleStates: Array<SwerveModuleState?>
    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    get() {
      val states: Array<SwerveModuleState?> = arrayOfNulls(4)
      for (i in 0..3) {
        states[i] = modules[i]!!.state
      }
      return states
    }

  private val modulePositions: Array<SwerveModulePosition?>
    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    get() {
      val states: Array<SwerveModulePosition?> = arrayOfNulls<SwerveModulePosition>(4)
      for (i in 0..3) {
        states[i] = modules[i]!!.modulePosition
      }
      return states
    }

  val chassisSpeeds: ChassisSpeeds
    /** Returns the measured chassis speeds of the robot. */
    get() = ChassisSpeeds(kinematics.toChassisSpeeds(moduleStates))

  val wheelRadiusCharacterizationPositions: DoubleArray
    /** Returns the position of each module in radians. */
    get() {
      val values = DoubleArray(4)
      for (i in 0..3) {
        values[i] = modules[i]!!.wheelRadiusCharacterizationPosition
      }
      return values
    }

  val ffCharacterizationVelocity: Double
    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    get() {
      var output = 0.0
      for (i in 0..3) {
        output += modules[i]!!.ffCharacterizationVelocity / 4.0
      }
      return output
    }

  var pose: Pose3d
    /** Returns the current odometry pose. */
    get() =
      if (RobotBase.isReal()) Pose3d(poseEstimator.estimatedPosition)
      else Pose3d(Pose2d(getSimulationPoseCallback.get()))

    /** Resets the current odometry pose. */
    set(pose) {
      resetSimulationPoseCallback.accept(pose.toPose2d().pose2d)
      poseEstimator.resetPosition(rawGyroRotation.rotation3d, modulePositions, pose.pose3d)
    }

  val rotation: Rotation3d
    /** Returns the current odometry pitch rotation. */
    get() = pose.rotation

  /** Adds a new timestamped vision measurement. */
  fun addVisionMeasurement(
    visionRobotPose: WPIPose3d?,
    timestampSeconds: Double,
    visionMeasurementStdDevs: Matrix<N4?, N1?>?
  ) {
    poseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStdDevs)
  }

  val maxLinearSpeedMetersPerSec: Double
    /** Returns the maximum linear speed in meters per sec. */
    get() = TunerConstants.kSpeedAt12Volts.inMetersPerSecond

  val maxAngularSpeedRadPerSec: Double
    /** Returns the maximum angular speed in radians per sec. */
    get() = maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

  companion object {
    // TunerConstants doesn't include these constants, so they are declared locally
    @JvmField
    val ODOMETRY_FREQUENCY: Double =
      if (CANBus(TunerConstants.CTREDrivetrainConstants.CANBusName).isNetworkFD) 250.0 else 100.0
    val DRIVE_BASE_RADIUS: Double =
      max(
        max(
          hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)
        ),
        max(
          hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        )
      )

    // PathPlanner config constants
    val PP_CONFIG: RobotConfig =
      RobotConfig(
        Constants.Universal.ROBOT_WEIGHT.inKilograms,
        Constants.Universal.ROBOT_MOI.inKilogramsMeterSquared,
        ModuleConfig(
          TunerConstants.FrontLeft.WheelRadius,
          TunerConstants.kSpeedAt12Volts.inMetersPerSecond,
          DrivetrainConstants.NITRILE_WHEEL_COF,
          DCMotor.getKrakenX60Foc(1)
            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
          TunerConstants.FrontLeft.SlipCurrent,
          1
        ),
        *(
          moduleTranslations
            .map { translation: Translation2d -> translation.translation2d }
            .toTypedArray()
          )
      )

    @JvmField val odometryLock: Lock = ReentrantLock()
    val moduleTranslations: Array<Translation2d>
      /** Returns an array of module translations. */
      get() =
        arrayOf(
          Translation2d(
            TunerConstants.FrontLeft.LocationX.meters,
            TunerConstants.FrontLeft.LocationY.meters
          ),
          Translation2d(
            TunerConstants.FrontRight.LocationX.meters,
            TunerConstants.FrontRight.LocationY.meters
          ),
          Translation2d(
            TunerConstants.BackLeft.LocationX.meters,
            TunerConstants.BackLeft.LocationY.meters
          ),
          Translation2d(
            TunerConstants.BackRight.LocationX.meters,
            TunerConstants.BackRight.LocationY.meters
          )
        )

    val mapleSimConfig: DriveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
        .withBumperSize(
          Meters.of(
            (DrivetrainConstants.DRIVETRAIN_LENGTH + DrivetrainConstants.BUMPER_WIDTH * 2)
              .inMeters
          ),
          Meters.of(
            (DrivetrainConstants.DRIVETRAIN_WIDTH + DrivetrainConstants.BUMPER_WIDTH * 2)
              .inMeters
          )
        )
        .withTrackLengthTrackWidth(
          Meters.of(
            TunerConstants.FrontLeft.LocationX.absoluteValue +
              TunerConstants.BackRight.LocationX.absoluteValue
          ),
          Meters.of(
            TunerConstants.FrontLeft.LocationY.absoluteValue +
              TunerConstants.BackRight.LocationY.absoluteValue
          )
        )
        .withRobotMass(Kilograms.of(Constants.Universal.ROBOT_WEIGHT.inKilograms))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModules(
          // FL, FR, BL, BR
          // reason we map the factories is in case we have seperate
          // modules (aka seperate ratios/constants) on some corners
          *(
            arrayOf(
              TunerConstants.FrontLeft,
              TunerConstants.FrontRight,
              TunerConstants.BackLeft,
              TunerConstants.BackRight
            )
              .map {
                SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  it.DriveMotorGearRatio,
                  it.SteerMotorGearRatio,
                  Volts.of(it.DriveFrictionVoltage),
                  Volts.of(it.SteerFrictionVoltage),
                  Meters.of(it.WheelRadius),
                  KilogramSquareMeters.of(it.SteerInertia),
                  DrivetrainConstants.NITRILE_WHEEL_COF
                )
              }
              .toTypedArray()
            )
        )
  }
}

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
package com.team4099.robot2026.subsystems.drivetrain

import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import com.team4099.lib.hal.Clock
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.DrivetrainConstants
import com.team4099.robot2026.config.constants.FieldConstants
import com.team4099.robot2026.config.constants.VisionConstants
import com.team4099.robot2026.util.AllianceFlipUtil
import com.team4099.robot2026.util.CustomLogger
import com.team4099.robot2026.util.Velocity2d
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.hypot
import kotlin.math.max
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Twist2d
import org.team4099.lib.kinematics.ChassisVelocities
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.inKilograms
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inMilliseconds
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.wpilib.command2.Command
import org.wpilib.command2.SubsystemBase
import org.wpilib.command2.sysid.SysIdRoutine
import org.wpilib.driverstation.RobotState
import org.wpilib.framework.RobotBase
import org.wpilib.math.estimator.SwerveDrivePoseEstimator
import org.wpilib.math.geometry.Pose2d as WPIPose2d
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.math.linalg.Matrix
import org.wpilib.math.numbers.N1
import org.wpilib.math.numbers.N3
import org.wpilib.math.system.DCMotor
import org.wpilib.sysid.SysIdRoutineLog
import org.wpilib.units.Units
import org.wpilib.units.Units.Seconds
import org.wpilib.units.Units.Volts
import org.wpilib.units.measure.Voltage
import org.wpilib.util.Alert

class Drive(
    private val gyroIO: GyroIO,
    val moduleIOs: Array<ModuleIO>,
) : SubsystemBase() {
  private val gyroInputs: GyroIO.GyroIOInputs = GyroIO.GyroIOInputs()
  private val modules = arrayOfNulls<Module>(4) // FL, FR, BL, BR
  private val sysId: SysIdRoutine
  private val gyroDisconnectedAlert =
      Alert(
          "gyroDisconnected", "Disconnected gyro, using kinematics as fallback.", Alert.Level.HIGH)

  private val kinematics: SwerveDriveKinematics =
      SwerveDriveKinematics(
          *(moduleTranslations
              .map { translation: Translation2d -> translation.translation2d }
              .toTypedArray()))
  private var rawGyroRotation: Angle = 0.radians
  private val lastModulePositions: Array<SwerveModulePosition?> = // For delta tracking
      arrayOf(
          SwerveModulePosition(),
          SwerveModulePosition(),
          SwerveModulePosition(),
          SwerveModulePosition())
  private val poseEstimator: SwerveDrivePoseEstimator =
      SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation.inRotation2ds,
          lastModulePositions,
          DrivetrainConstants.INITIAL_SIM_POSE,
          DrivetrainConstants.STATE_STDEVS,
          VisionConstants.singleTagStdDevs)

  var targetSpeeds = ChassisVelocities()
    private set

  init {
    modules[0] = Module(moduleIOs[0], 0, DrivetrainConstants.tunerConstants.FrontLeft)
    modules[1] = Module(moduleIOs[1], 1, DrivetrainConstants.tunerConstants.FrontRight)
    modules[2] = Module(moduleIOs[2], 2, DrivetrainConstants.tunerConstants.BackLeft)
    modules[3] = Module(moduleIOs[3], 3, DrivetrainConstants.tunerConstants.BackRight)

    // Start odometry thread
    PhoenixOdometryThread.instance.start()

    // Configure SysId
    sysId =
        SysIdRoutine(
            SysIdRoutine.Config(Volts.of(1.0) / Seconds.of(1.0), Volts.of(7.0), Seconds.of(10.0)) {
                state: SysIdRoutineLog.State ->
              CustomLogger.recordOutput("Drive/SysIdState", state.toString())
            },
            SysIdRoutine.Mechanism(
                { voltage: Voltage -> runCharacterization(voltage.`in`(Units.Volts)) }, null, this))
  }

  override fun periodic() {
    val startTime = Clock.epochTime

    odometryLock.lock() // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs)
    CustomLogger.processInputs("Drive/Gyro", gyroInputs)
    for (module in modules) {
      module!!.periodic()
    }
    odometryLock.unlock()

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (module in modules) {
        module!!.stop()
      }
    }

    // Log empty setpoint states when disabled
    if (RobotState.isDisabled()) {
      CustomLogger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleVelocity>())
      CustomLogger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleVelocity>())
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
                modulePositions[moduleIndex]!!.distance -
                    lastModulePositions[moduleIndex]!!.distance,
                modulePositions[moduleIndex]!!.angle)
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i]
      } else {
        // Use the angle delta from the kinematics and module deltas
        val twist = Twist2d(kinematics.toTwist2d(*moduleDeltas))
        rawGyroRotation += twist.dtheta
      }

      // Apply update
      poseEstimator.updateWithTime(
          sampleTimestamps[i], rawGyroRotation.inRotation2ds, modulePositions)
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && RobotBase.isReal())

    CustomLogger.recordOutput("Odometry/pose", pose.pose2d)
    CustomLogger.recordOutput("SwerveChassisSpeeds/Measured", chassisSpeeds.chassisVelocitiesWPILIB)

    Logger.recordOutput("SwerveStates/Measured", *moduleStates)

    CustomLogger.recordOutput("Odometry/inAllianceZone", FieldConstants.inAllianceZone(pose))
    CustomLogger.recordOutput(
        "Odometry/inTrenchAllianceZone", FieldConstants.inTrenchAllianceZone(pose))

    CustomLogger.recordOutput(
        "LoggedRobot/Subsystems/DriveLoopTimeMS", (Clock.epochTime - startTime).inMilliseconds)
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  fun runSpeeds(speeds: ChassisVelocities, flipIfRed: Boolean = true) {
    targetSpeeds = speeds

    val flippedSpeeds =
        if (flipIfRed && AllianceFlipUtil.shouldFlip())
            ChassisVelocities(-speeds.vx, -speeds.vy, speeds.omega)
        else speeds
    // Calculate module setpoints
    val discreteSpeeds = flippedSpeeds.discretize(Constants.Universal.LOOP_PERIOD_TIME)
    val setpointStates: Array<SwerveModuleVelocity> =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(discreteSpeeds.chassisVelocitiesWPILIB),
            DrivetrainConstants.tunerConstants.kSpeedAt12Volts.inMetersPerSecond)

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds.chassisVelocitiesWPILIB)

    // Send setpoints to modules
    val optimizedStates = Array(4) { i -> modules[i]!!.runSetpoint(setpointStates[i]) }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", *optimizedStates)
  }

  fun runTranslationWhileKeepingRotation(vels: Velocity2d, flipIfRed: Boolean = true) {
    runSpeeds(ChassisVelocities(vels.x, vels.y, targetSpeeds.omega), flipIfRed)
  }

  fun runRotationWhileKeepingTranslation(omega: AngularVelocity, flipIfRed: Boolean = true) {
    runSpeeds(ChassisVelocities(targetSpeeds.vx, targetSpeeds.vy, omega), flipIfRed)
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
    runSpeeds(ChassisVelocities())
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
        *(headings.map { heading: Angle -> heading.inRotation2ds }.toTypedArray()))
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

  private val moduleStates: Array<SwerveModuleVelocity?>
    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    get() {
      val states: Array<SwerveModuleVelocity?> = arrayOfNulls(4)
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

  val chassisSpeeds: ChassisVelocities
    /** Returns the measured chassis speeds of the robot. */
    get() = ChassisVelocities(kinematics.toChassisVelocities(*moduleStates))

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

  var pose: Pose2d
    /** Returns the current odometry pose. */
    get() = Pose2d(poseEstimator.estimatedPosition)
    /** Resets the current odometry pose. */
    set(pose) {
      poseEstimator.resetPosition(rawGyroRotation.inRotation2ds, modulePositions, pose.pose2d)
    }

  val rotation: Angle
    /** Returns the current odometry pitch rotation. */
    get() = pose.rotation

  /** Adds a new timestamped vision measurement. */
  fun addVisionMeasurement(
      visionRobotPose: WPIPose2d?,
      timestampSeconds: Double,
      visionMeasurementStdDevs: Matrix<N3?, N1?>?
  ) {
    poseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStdDevs)
  }

  val maxLinearSpeedMetersPerSec: Double
    /** Returns the maximum linear speed in meters per sec. */
    get() = DrivetrainConstants.tunerConstants.kSpeedAt12Volts.inMetersPerSecond

  val maxAngularSpeedRadPerSec: Double
    /** Returns the maximum angular speed in radians per sec. */
    get() = maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

  companion object {
    // TunerConstants doesn't include these constants, so they are declared locally
    @JvmField val ODOMETRY_FREQUENCY: Double = if (PhoenixOdometryThread.isCANFD) 250.0 else 100.0

    val DRIVE_BASE_RADIUS: Double =
        max(
            max(
                hypot(
                    DrivetrainConstants.tunerConstants.FrontLeft.LocationX,
                    DrivetrainConstants.tunerConstants.FrontLeft.LocationY),
                hypot(
                    DrivetrainConstants.tunerConstants.FrontRight.LocationX,
                    DrivetrainConstants.tunerConstants.FrontRight.LocationY)),
            max(
                hypot(
                    DrivetrainConstants.tunerConstants.BackLeft.LocationX,
                    DrivetrainConstants.tunerConstants.BackLeft.LocationY),
                hypot(
                    DrivetrainConstants.tunerConstants.BackRight.LocationX,
                    DrivetrainConstants.tunerConstants.BackRight.LocationY)))

    // PathPlanner config constants
    val PP_CONFIG: RobotConfig =
        RobotConfig(
            Constants.Universal.ROBOT_WEIGHT.inKilograms,
            Constants.Universal.ROBOT_MOI_KG_METERS_SQUARED,
            ModuleConfig(
                DrivetrainConstants.tunerConstants.FrontLeft.WheelRadius,
                DrivetrainConstants.tunerConstants.kSpeedAt12Volts.inMetersPerSecond,
                DrivetrainConstants.CURRENT_COF,
                DCMotor.getKrakenX60Foc(1)
                    .withReduction(
                        DrivetrainConstants.tunerConstants.FrontLeft.DriveMotorGearRatio),
                DrivetrainConstants.tunerConstants.FrontLeft.SlipCurrent,
                1),
            *(moduleTranslations
                .map { translation: Translation2d -> translation.translation2d }
                .toTypedArray()))

    @JvmField val odometryLock: Lock = ReentrantLock()
    val moduleTranslations: Array<Translation2d>
      /** Returns an array of module translations. */
      get() =
          arrayOf(
              Translation2d(
                  DrivetrainConstants.tunerConstants.FrontLeft.LocationX.meters,
                  DrivetrainConstants.tunerConstants.FrontLeft.LocationY.meters),
              Translation2d(
                  DrivetrainConstants.tunerConstants.FrontRight.LocationX.meters,
                  DrivetrainConstants.tunerConstants.FrontRight.LocationY.meters),
              Translation2d(
                  DrivetrainConstants.tunerConstants.BackLeft.LocationX.meters,
                  DrivetrainConstants.tunerConstants.BackLeft.LocationY.meters),
              Translation2d(
                  DrivetrainConstants.tunerConstants.BackRight.LocationX.meters,
                  DrivetrainConstants.tunerConstants.BackRight.LocationY.meters))
  }
}

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

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.perSecond

class Module(
  private val io: ModuleIO,
  private val index: Int,
  private val constants:
    SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>
) {
  private val inputs: ModuleIO.ModuleIOInputs = ModuleIO.ModuleIOInputs()

  private val driveDisconnectedAlert =
    Alert("Disconnected drive motor on module $index.", Alert.AlertType.kError)
  private val turnDisconnectedAlert =
    Alert("Disconnected turn motor on module $index.", Alert.AlertType.kError)
  private val turnEncoderDisconnectedAlert =
    Alert("Disconnected turn encoder on module $index.", Alert.AlertType.kError)

  /** Returns the module positions received this cycle. */
  var odometryPositions: Array<SwerveModulePosition?> = arrayOf()
    private set

  fun periodic() {
    io.updateInputs(inputs)
    Logger.processInputs("Drive/Module$index", inputs)

    // Calculate positions for odometry
    val sampleCount: Int = inputs.odometryTimestamps.size // All signals are sampled together
    odometryPositions = arrayOfNulls(sampleCount)
    for (i in 0 until sampleCount) {
      val position = (inputs.odometryDrivePositions[i].inRadians * constants.WheelRadius).meters
      val angle: Angle = inputs.odometryTurnPositions[i]
      odometryPositions[i] = SwerveModulePosition(position.inMeters, angle.inRotation2ds)
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected)
    turnDisconnectedAlert.set(!inputs.turnConnected)
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected)
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  fun runSetpoint(state: SwerveModuleState) {
    // Optimize velocity setpoint
    state.optimize(angle.inRotation2ds)
    state.cosineScale(inputs.turnPosition.inRotation2ds)

    // Apply setpoints
    val desiredVoltage =
      state.speedMetersPerSecond / constants.SpeedAt12Volts *
        DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE.inVolts
    //    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius)
    io.setDriveOpenLoop(desiredVoltage)
    io.setTurnPosition(state.angle)
  }

  fun pointWheelsAt(angle: Angle) {
    val state = SwerveModuleState(0.0, angle.inRotation2ds)
    state.optimize(angle.inRotation2ds)

    io.setTurnPosition(state.angle)
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  fun runCharacterization(output: Double) {
    io.setDriveOpenLoop(output)
    io.setTurnPosition(Rotation2d())
  }

  /** Disables all outputs to motors. */
  fun stop() {
    io.setDriveOpenLoop(0.0)
    io.setTurnOpenLoop(0.0)
  }

  val angle: Angle
    /** Returns the current turn angle of the module. */
    get() = inputs.turnPosition

  val position: Length
    /** Returns the current drive position of the module in meters. */
    get() = (inputs.drivePosition.inRadians * constants.WheelRadius).meters

  val velocity: LinearVelocity
    /** Returns the current drive velocity of the module in meters per second. */
    get() = (inputs.driveVelocity.inRadiansPerSecond * constants.WheelRadius).meters.perSecond

  val modulePosition: SwerveModulePosition
    /** Returns the module modulePosition (turn angle and drive modulePosition). */
    get() = SwerveModulePosition(position.inMeters, angle.inRotation2ds)

  val state: SwerveModuleState
    /** Returns the module state (turn angle and drive velocity). */
    get() = SwerveModuleState(velocity.inMetersPerSecond, angle.inRotation2ds)

  val odometryTimestamps: DoubleArray
    /** Returns the timestamps of the samples received this cycle. */
    get() = inputs.odometryTimestamps

  val wheelRadiusCharacterizationPosition: Double
    /** Returns the module modulePosition in radians. */
    get() = inputs.drivePosition.inRadians

  val ffCharacterizationVelocity: Double
    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    get() = inputs.driveVelocity.inRotationsPerSecond
}

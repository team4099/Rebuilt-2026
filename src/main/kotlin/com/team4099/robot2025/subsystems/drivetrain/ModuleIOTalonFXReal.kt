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
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import org.team4099.lib.units.derived.rotations
import java.util.Queue

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
class ModuleIOTalonFXReal(
  constants:
    SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>
) : ModuleIOTalonFX(constants) {
  // Create timestamp queue
  private val timestampQueue: Queue<Double> = PhoenixOdometryThread.instance.makeTimestampQueue()
  private val drivePositionQueue: Queue<Double> =
    PhoenixOdometryThread.instance.registerSignal(driveTalon.position)
  private val turnPositionQueue: Queue<Double> =
    PhoenixOdometryThread.instance.registerSignal(turnTalon.position)

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    super.updateInputs(inputs)

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue.map { value: Double -> value }.toDoubleArray()
    inputs.odometryDrivePositions =
      drivePositionQueue.map { value: Double -> value.rotations }.toTypedArray()
    inputs.odometryTurnPositions =
      turnPositionQueue.map { value: Double -> value.rotations }.toTypedArray()
    timestampQueue.clear()
    drivePositionQueue.clear()
    turnPositionQueue.clear()
  }

  companion object {
    fun generateModules(): Array<ModuleIO> {
      return arrayOf(
        ModuleIOTalonFXReal(TunerConstants.FrontLeft),
        ModuleIOTalonFXReal(TunerConstants.FrontRight),
        ModuleIOTalonFXReal(TunerConstants.BackLeft),
        ModuleIOTalonFXReal(TunerConstants.BackRight)
      )
    }
  }
}

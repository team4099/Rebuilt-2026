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
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.ModuleIO.ModuleIOInputs
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeters
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianSeconds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import kotlin.math.sign

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
class ModuleIOSim(
  constants:
    SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>
) : ModuleIO {
  // Create drive and turn sim models
  private val driveSim: DCMotorSim =
    DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio
      ),
      DRIVE_GEARBOX
    )
  private val turnSim: DCMotorSim =
    DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio
      ),
      TURN_GEARBOX
    )

  private var driveClosedLoop = false
  private var turnClosedLoop = false
  private val driveController = PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD)
  private val turnController = PIDController(TURN_KP, TURN_KI, TURN_KD)
  private var driveFFVolts = 0.0
  private var driveAppliedVolts = 0.0
  private var turnAppliedVolts = 0.0

  init {
    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI)
  }

  override fun updateInputs(inputs: ModuleIOInputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
        driveFFVolts + driveController.calculate(driveSim.angularVelocityRadPerSec)
    } else {
      driveController.reset()
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.angularPositionRad)
    } else {
      turnController.reset()
    }

    // Update simulation state
    driveSim.inputVoltage =
      MathUtil.clamp(
        driveAppliedVolts,
        -DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE.inVolts,
        DrivetrainConstants.DRIVE_COMPENSATION_VOLTAGE.inVolts
      )
    turnSim.inputVoltage =
      MathUtil.clamp(
        turnAppliedVolts,
        -DrivetrainConstants.STEERING_COMPENSATION_VOLTAGE.inVolts,
        DrivetrainConstants.STEERING_COMPENSATION_VOLTAGE.inVolts
      )
    driveSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    turnSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    // Update drive inputs
    inputs.driveConnected = true
    inputs.drivePosition = driveSim.angularPositionRad.radians
    inputs.driveVelocity = driveSim.angularVelocityRadPerSec.radians.perSecond
    inputs.driveAppliedVoltage = driveAppliedVolts.volts
    inputs.driveCurrent = driveSim.currentDrawAmps.amps.absoluteValue

    // Update turn inputs
    inputs.turnConnected = true
    inputs.turnEncoderConnected = true
    inputs.turnAbsolutePosition = turnSim.angularPositionRad.radians
    inputs.turnPosition = turnSim.angularPositionRad.radians
    inputs.turnVelocity = turnSim.angularVelocityRadPerSec.radians.perSecond
    inputs.turnAppliedVoltage = turnAppliedVolts.volts
    inputs.turnCurrent = turnSim.currentDrawAmps.amps.absoluteValue

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = doubleArrayOf(Timer.getFPGATimestamp())
    inputs.odometryDrivePositions = arrayOf(inputs.drivePosition)
    inputs.odometryTurnPositions = arrayOf(inputs.turnPosition)
  }

  override fun setDriveOpenLoop(output: Double) {
    driveClosedLoop = false
    driveAppliedVolts = output
  }

  override fun setTurnOpenLoop(output: Double) {
    turnClosedLoop = false
    turnAppliedVolts = output
  }

  override fun setDriveVelocity(velocityRadPerSec: Double) {
    driveClosedLoop = true
    driveFFVolts = DRIVE_KS * sign(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec
    driveController.setpoint = velocityRadPerSec
  }

  override fun setTurnPosition(rotation: Rotation2d) {
    turnClosedLoop = true
    turnController.setpoint = rotation.radians
  }

  override fun toggleBrakeMode(brake: NeutralModeValue) {}

  companion object {
    // TunerConstants doesn't support separate sim constants, so they are declared locally
    private val DRIVE_KP = DrivetrainConstants.PID.SIM_DRIVE_KP.inVoltsPerMetersPerSecond
    private val DRIVE_KI = DrivetrainConstants.PID.SIM_DRIVE_KI.inVoltsPerMeters
    private val DRIVE_KD = DrivetrainConstants.PID.SIM_DRIVE_KD.inVoltsPerMetersPerSecondPerSecond
    private val DRIVE_KS = DrivetrainConstants.PID.SIM_DRIVE_KS.inVolts
    private val DRIVE_KV = DrivetrainConstants.PID.SIM_DRIVE_KV.inVoltsPerMeterPerSecond
    private val TURN_KP = DrivetrainConstants.PID.SIM_STEERING_KP.inVoltsPerRadian
    private val TURN_KI = DrivetrainConstants.PID.SIM_STEERING_KI.inVoltsPerRadianSeconds
    private val TURN_KD = DrivetrainConstants.PID.SIM_STEERING_KD.inVoltsPerRadianPerSecond
    private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)

    fun generateModules(): Array<ModuleIO> {
      return arrayOf(
        ModuleIOSim(TunerConstants.FrontLeft),
        ModuleIOSim(TunerConstants.FrontRight),
        ModuleIOSim(TunerConstants.BackLeft),
        ModuleIOSim(TunerConstants.BackRight)
      )
    }
  }
}

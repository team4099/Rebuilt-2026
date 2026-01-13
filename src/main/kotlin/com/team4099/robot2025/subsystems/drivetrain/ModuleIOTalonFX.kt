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

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.TorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.team4099.lib.phoenix6.PhoenixUtil.tryUntilOk
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import java.util.Queue

abstract class ModuleIOTalonFX(
  val constants:
    SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>
) : ModuleIO {
  protected val driveTalon: TalonFX
  protected val turnTalon: TalonFX
  protected val cancoder: CANcoder

  protected val voltageRequest = VoltageOut(0.0).withEnableFOC(true)
  protected val positionVoltageRequest = MotionMagicVoltage(0.0).withEnableFOC(true)
  protected val velocityVoltageRequest = VelocityVoltage(0.0).withEnableFOC(true)

  // Torque-current control requests
  protected val torqueCurrentRequest: TorqueCurrentFOC = TorqueCurrentFOC(0.0)
  protected val positionTorqueCurrentRequest: PositionTorqueCurrentFOC =
    PositionTorqueCurrentFOC(0.0)
  protected val velocityTorqueCurrentRequest: VelocityTorqueCurrentFOC =
    VelocityTorqueCurrentFOC(0.0)

  // Inputs from drive motor
  private val drivePosition: StatusSignal<Angle>
  private val drivePositionQueue: Queue<Double>
  private val driveVelocity: StatusSignal<AngularVelocity>
  private val driveAppliedVolts: StatusSignal<Voltage>
  private val driveCurrent: StatusSignal<Current>

  // Inputs from turn motor
  private val turnAbsolutePosition: StatusSignal<Angle>
  private val turnPosition: StatusSignal<Angle>
  private val turnPositionQueue: Queue<Double>
  private val turnVelocity: StatusSignal<AngularVelocity>
  private val turnAppliedVolts: StatusSignal<Voltage>
  private val turnCurrent: StatusSignal<Current>

  // Connection debouncers
  private val driveConnectedDebounce: Debouncer = Debouncer(0.5)
  private val turnConnectedDebounce: Debouncer = Debouncer(0.5)
  private val turnEncoderConnectedDebounce: Debouncer = Debouncer(0.5)

  init {
    driveTalon = TalonFX(constants.DriveMotorId, TunerConstants.CTREDrivetrainConstants.CANBusName)
    turnTalon = TalonFX(constants.SteerMotorId, TunerConstants.CTREDrivetrainConstants.CANBusName)
    cancoder = CANcoder(constants.EncoderId, TunerConstants.CTREDrivetrainConstants.CANBusName)

    // Configure drive motor
    val driveConfig = constants.DriveMotorInitialConfigs!!
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
    driveConfig.Slot0 = constants.DriveMotorGains
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true
    driveConfig.MotorOutput.Inverted =
      if (constants.DriveMotorInverted) InvertedValue.Clockwise_Positive
      else InvertedValue.CounterClockwise_Positive
    tryUntilOk(5) { driveTalon.configurator.apply(driveConfig, 0.25) }
    tryUntilOk(5) { driveTalon.setPosition(0.0, 0.25) }

    // Configure turn motor
    val turnConfig = TalonFXConfiguration()
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake
    turnConfig.Slot0 = constants.SteerMotorGains
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId
    turnConfig.Feedback.FeedbackSensorSource =
      when (constants.FeedbackSource) {
        SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder ->
          FeedbackSensorSourceValue.RemoteCANcoder
        SwerveModuleConstants.SteerFeedbackType.FusedCANcoder ->
          FeedbackSensorSourceValue.FusedCANcoder
        SwerveModuleConstants.SteerFeedbackType.SyncCANcoder ->
          FeedbackSensorSourceValue.SyncCANcoder
        else -> throw RuntimeException("Using an unsupported swerve configuration")
      }
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio

    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio
    turnConfig.MotionMagic.MotionMagicAcceleration = 100.0 / constants.SteerMotorGearRatio / 0.100
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1

    turnConfig.ClosedLoopGeneral.ContinuousWrap = true

    turnConfig.MotorOutput.Inverted =
      if (constants.SteerMotorInverted) InvertedValue.Clockwise_Positive
      else InvertedValue.CounterClockwise_Positive
    tryUntilOk(5) { turnTalon.configurator.apply(turnConfig, 0.25) }

    // Configure CANCoder
    val cancoderConfig = constants.EncoderInitialConfigs!!
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset
    cancoderConfig.MagnetSensor.SensorDirection =
      if (constants.EncoderInverted) SensorDirectionValue.Clockwise_Positive
      else SensorDirectionValue.CounterClockwise_Positive
    cancoder.configurator.apply(cancoderConfig, 0.25)

    // Create drive status signals
    drivePosition = driveTalon.position
    drivePositionQueue = PhoenixOdometryThread.instance.registerSignal(driveTalon.position)
    driveVelocity = driveTalon.velocity
    driveAppliedVolts = driveTalon.motorVoltage
    driveCurrent = driveTalon.statorCurrent

    // Create turn status signals
    turnAbsolutePosition = cancoder.absolutePosition
    turnPosition = turnTalon.position
    turnPositionQueue = PhoenixOdometryThread.instance.registerSignal(turnTalon.position)
    turnVelocity = turnTalon.velocity
    turnAppliedVolts = turnTalon.motorVoltage
    turnCurrent = turnTalon.statorCurrent

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition)
    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      driveVelocity,
      driveAppliedVolts,
      driveCurrent,
      turnAbsolutePosition,
      turnVelocity,
      turnAppliedVolts,
      turnCurrent
    )
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon)
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    // Refresh all signals
    val driveStatus =
      BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent)
    val turnStatus =
      BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent)
    val turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition)

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK)
    inputs.drivePosition = drivePosition.valueAsDouble.rotations
    inputs.driveVelocity = driveVelocity.valueAsDouble.rotations.perSecond
    inputs.driveAppliedVoltage = driveAppliedVolts.valueAsDouble.volts
    inputs.driveCurrent = driveCurrent.valueAsDouble.amps

    // Update turn inputs
    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK)
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK)
    inputs.turnAbsolutePosition = turnAbsolutePosition.valueAsDouble.rotations
    inputs.turnPosition = turnPosition.valueAsDouble.rotations
    inputs.turnVelocity = turnVelocity.valueAsDouble.rotations.perSecond
    inputs.turnAppliedVoltage = turnAppliedVolts.valueAsDouble.volts
    inputs.turnCurrent = turnCurrent.valueAsDouble.amps
  }

  override fun setDriveOpenLoop(output: Double) {
    driveTalon.setControl(voltageRequest.withOutput(output))
  }

  override fun setTurnOpenLoop(output: Double) {
    turnTalon.setControl(voltageRequest.withOutput(output))
  }

  override fun setDriveVelocity(velocityRadPerSec: Double) {
    val velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec)
    driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec))
  }

  override fun setTurnPosition(rotation: Rotation2d) {
    turnTalon.setControl(positionVoltageRequest.withPosition(rotation.rotations))
  }

  override fun toggleBrakeMode(brake: NeutralModeValue) {
    driveTalon.configurator.apply(
      constants.DriveMotorInitialConfigs!!.MotorOutput.withNeutralMode(brake)
    )
    turnTalon.configurator.apply(
      constants.SteerMotorInitialConfigs!!.MotorOutput.withNeutralMode(brake)
    )
  }
}

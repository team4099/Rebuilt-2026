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
import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import edu.wpi.first.units.measure.AngularVelocity
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.perSecond
import java.util.Queue

/** IO implementation for Pigeon 2. */
object GyroIOPigeon2 : GyroIO {
  private val pigeon: Pigeon2 =
    Pigeon2(
      TunerConstants.CTREDrivetrainConstants.Pigeon2Id,
      CANBus(TunerConstants.CTREDrivetrainConstants.CANBusName)
    )

  private val roll: StatusSignal<edu.wpi.first.units.measure.Angle> = pigeon.roll
  private val rollPositionQueue: Queue<Double>
  private val rollTimestampQueue: Queue<Double>
  private val pitch: StatusSignal<edu.wpi.first.units.measure.Angle> = pigeon.pitch
  private val pitchPositionQueue: Queue<Double>
  private val pitchTimestampQueue: Queue<Double>
  private val yaw: StatusSignal<edu.wpi.first.units.measure.Angle> = pigeon.yaw
  private val yawPositionQueue: Queue<Double>
  private val yawTimestampQueue: Queue<Double>
  private val rollVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityXWorld
  private val pitchVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityYWorld
  private val yawVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityZWorld

  init {
    pigeon.configurator.apply(Pigeon2Configuration())
    pigeon.configurator.setYaw(0.0)
    roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)
    pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)

    rollVelocity.setUpdateFrequency(50.0)
    pitchVelocity.setUpdateFrequency(50.0)
    yawVelocity.setUpdateFrequency(50.0)
    pigeon.optimizeBusUtilization()

    rollTimestampQueue = PhoenixOdometryThread.instance.makeTimestampQueue()
    rollPositionQueue = PhoenixOdometryThread.instance.registerSignal(pigeon.roll)
    pitchTimestampQueue = PhoenixOdometryThread.instance.makeTimestampQueue()
    pitchPositionQueue = PhoenixOdometryThread.instance.registerSignal(pigeon.pitch)
    yawTimestampQueue = PhoenixOdometryThread.instance.makeTimestampQueue()
    yawPositionQueue = PhoenixOdometryThread.instance.registerSignal(pigeon.yaw)
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected =
      BaseStatusSignal.refreshAll(roll, rollVelocity, pitch, pitchVelocity, yaw, yawVelocity) ==
      StatusCode.OK

    inputs.rollPosition = roll.valueAsDouble.degrees
    inputs.rollVelocity = rollVelocity.valueAsDouble.degrees.perSecond
    inputs.pitchPosition = pitch.valueAsDouble.degrees
    inputs.pitchVelocity = pitchVelocity.valueAsDouble.degrees.perSecond
    inputs.yawPosition = yaw.valueAsDouble.degrees
    inputs.yawVelocity = yawVelocity.valueAsDouble.degrees.perSecond

    inputs.odometryRollTimestamps =
      rollTimestampQueue.map { value: Double -> value }.toDoubleArray()
    inputs.odometryRollPositions =
      rollPositionQueue.map { value: Double -> value.degrees }.toTypedArray()
    inputs.odometryPitchTimestamps =
      pitchTimestampQueue.map { value: Double -> value }.toDoubleArray()
    inputs.odometryPitchPositions =
      pitchPositionQueue.map { value: Double -> value.degrees }.toTypedArray()
    inputs.odometryYawTimestamps = yawTimestampQueue.map { value: Double -> value }.toDoubleArray()
    inputs.odometryYawPositions =
      yawPositionQueue.map { value: Double -> value.degrees }.toTypedArray()

    rollTimestampQueue.clear()
    rollPositionQueue.clear()
    pitchTimestampQueue.clear()
    pitchPositionQueue.clear()
    yawTimestampQueue.clear()
    yawPositionQueue.clear()
  }
}

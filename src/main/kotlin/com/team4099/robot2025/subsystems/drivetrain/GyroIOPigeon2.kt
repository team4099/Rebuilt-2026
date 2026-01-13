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
      TunerConstants.CTREDrivetrainConstants.CANBusName
    )
  private val yaw: StatusSignal<edu.wpi.first.units.measure.Angle> = pigeon.yaw
  private val yawPositionQueue: Queue<Double>
  private val yawTimestampQueue: Queue<Double>
  private val yawVelocity: StatusSignal<AngularVelocity> = pigeon.angularVelocityZWorld

  init {
    pigeon.configurator.apply(Pigeon2Configuration())
    pigeon.configurator.setYaw(0.0)
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY)
    yawVelocity.setUpdateFrequency(50.0)
    pigeon.optimizeBusUtilization()
    yawTimestampQueue = PhoenixOdometryThread.instance.makeTimestampQueue()
    yawPositionQueue = PhoenixOdometryThread.instance.registerSignal(pigeon.yaw)
  }

  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
    inputs.yawPosition = yaw.valueAsDouble.degrees
    inputs.yawVelocity = yawVelocity.valueAsDouble.degrees.perSecond

    inputs.odometryYawTimestamps = yawTimestampQueue.map { value: Double -> value }.toDoubleArray()
    inputs.odometryYawPositions =
      yawPositionQueue.map { value: Double -> value.degrees }.toTypedArray()
    yawTimestampQueue.clear()
    yawPositionQueue.clear()
  }
}

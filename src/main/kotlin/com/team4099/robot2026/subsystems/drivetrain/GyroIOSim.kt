package com.team4099.robot2026.subsystems.drivetrain

import com.team4099.lib.phoenix6.PhoenixUtil
import org.ironmaple.simulation.drivesims.GyroSimulation
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond
import org.wpilib.units.Units.RadiansPerSecond

class GyroIOSim(val gyroSimulation: GyroSimulation) : GyroIO {
  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = true
    inputs.yawPosition = gyroSimulation.gyroReading.radians.radians
    inputs.yawVelocity =
        gyroSimulation.measuredAngularVelocity.`in`(RadiansPerSecond).radians.perSecond

    inputs.odometryYawTimestamps = PhoenixUtil.simulationOdometryTimeStamps
    inputs.odometryYawPositions =
        gyroSimulation.cachedGyroReadings.map { it.radians.radians }.toTypedArray()

    inputs.odometryRollPositions = Array(gyroSimulation.cachedGyroReadings.size) { 0.radians }
    inputs.odometryPitchPositions = Array(gyroSimulation.cachedGyroReadings.size) { 0.radians }
  }
}

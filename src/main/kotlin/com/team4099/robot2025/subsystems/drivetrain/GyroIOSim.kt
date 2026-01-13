package com.team4099.robot2025.subsystems.drivetrain

import com.team4099.lib.phoenix6.PhoenixUtil
import edu.wpi.first.units.Units.RadiansPerSecond
import org.ironmaple.simulation.drivesims.GyroSimulation
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

class GyroIOSim(val gyroSimulation: GyroSimulation) : GyroIO {
  override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
    inputs.connected = true
    inputs.yawPosition = gyroSimulation.gyroReading.radians.radians
    inputs.yawVelocity =
      gyroSimulation.measuredAngularVelocity.`in`(RadiansPerSecond).radians.perSecond

    inputs.odometryYawTimestamps = PhoenixUtil.simulationOdometryTimeStamps
    inputs.odometryYawPositions =
      gyroSimulation.cachedGyroReadings.map { it.radians.radians }.toTypedArray()
  }
}

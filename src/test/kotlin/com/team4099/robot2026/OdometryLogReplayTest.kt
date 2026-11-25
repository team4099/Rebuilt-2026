package com.team4099.robot2026

import com.team4099.robot2026.subsystems.drivetrain.GyroIO
import com.team4099.robot2026.subsystems.drivetrain.ModuleIO
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Test
import org.littletonrobotics.junction.LogTable
import org.team4099.lib.units.derived.radians

class OdometryLogReplayTest {
  @Test
  fun `module and gyro odometry samples survive log replay`() {
    val module = ModuleIO.ModuleIOInputs()
    module.odometryTimestamps = doubleArrayOf(1.0, 1.02)
    module.odometryDrivePositions = arrayOf(2.0.radians, 2.5.radians)
    module.odometryTurnPositions = arrayOf(0.1.radians, 0.2.radians)
    val moduleLog = LogTable(0)
    module.toLog(moduleLog)
    val replayedModule = ModuleIO.ModuleIOInputs()
    replayedModule.fromLog(moduleLog)
    assertArrayEquals(module.odometryTimestamps, replayedModule.odometryTimestamps)
    assertArrayEquals(
        doubleArrayOf(2.0, 2.5),
        replayedModule.odometryDrivePositions.map { it.value }.toDoubleArray())
    assertArrayEquals(
        doubleArrayOf(0.1, 0.2),
        replayedModule.odometryTurnPositions.map { it.value }.toDoubleArray())

    val gyro = GyroIO.GyroIOInputs()
    gyro.odometryYawTimestamps = doubleArrayOf(1.0, 1.02)
    gyro.odometryYawPositions = arrayOf(0.3.radians, 0.4.radians)
    val gyroLog = LogTable(0)
    gyro.toLog(gyroLog)
    val replayedGyro = GyroIO.GyroIOInputs()
    replayedGyro.fromLog(gyroLog)
    assertArrayEquals(gyro.odometryYawTimestamps, replayedGyro.odometryYawTimestamps)
    assertArrayEquals(
        doubleArrayOf(0.3, 0.4), replayedGyro.odometryYawPositions.map { it.value }.toDoubleArray())
  }
}

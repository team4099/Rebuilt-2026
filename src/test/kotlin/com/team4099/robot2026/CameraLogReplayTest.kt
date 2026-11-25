package com.team4099.robot2026

import com.team4099.robot2026.subsystems.vision.camera.CameraIO
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.littletonrobotics.junction.LogTable
import org.photonvision.targeting.PhotonTrackedTarget
import org.wpilib.hardware.hal.HAL
import org.wpilib.math.geometry.Rotation3d
import org.wpilib.math.geometry.Transform3d
import org.wpilib.math.geometry.Translation3d

class CameraLogReplayTest {
  @Test
  fun `logged camera targets round trip without phantom detections`() {
    HAL.initialize()
    val empty = CameraIO.CameraInputs()
    val emptyTable = LogTable(0)
    empty.toLog(emptyTable)
    val emptyReplay = CameraIO.CameraInputs()
    emptyReplay.fromLog(emptyTable)
    assertEquals(0, emptyReplay.cameraTargets.size)

    val target = PhotonTrackedTarget()
    target.fiducialId = 7
    target.yaw = 11.0
    target.pitch = -4.0
    target.skew = 8.0
    target.area = 3.0
    target.poseAmbiguity = 0.12
    target.bestCameraToTarget = Transform3d(Translation3d(1.0, 2.0, 3.0), Rotation3d())
    val source = CameraIO.CameraInputs()
    source.cameraTargets.add(target)
    val table = LogTable(0)
    source.toLog(table)
    val replay = CameraIO.CameraInputs()
    replay.fromLog(table)
    assertEquals(1, replay.cameraTargets.size)
    val actual = replay.cameraTargets.single()
    assertEquals(7, actual.fiducialId)
    assertEquals(11.0, actual.yaw)
    assertEquals(-4.0, actual.pitch)
    assertEquals(8.0, actual.skew)
    assertEquals(0.12, actual.poseAmbiguity)
    assertEquals(1.0, actual.bestCameraToTarget.x)
  }
}

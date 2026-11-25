package com.team4099.robot2026

import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.subsystems.superstructure.feeder.Feeder
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIO
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.perSecond
import org.wpilib.hardware.hal.HAL

class FeederReadinessTest {
  @Test
  fun `feeder is ready only near requested speed`() {
    System.setProperty("robot", "compbot")
    System.setProperty("tuning", "f")
    HAL.initialize()
    val feeder = Feeder(object : FeederIO {})
    feeder.currentRequest = Request.FeederRequest.TargetVelocity(25.0.rotations.perSecond)
    feeder.inputs.feederVelocity = 0.0.rotations.perSecond
    assertFalse(feeder.isAtTargetedVelocity)
    feeder.inputs.feederVelocity = 25.0.rotations.perSecond
    assertTrue(feeder.isAtTargetedVelocity)
    feeder.inputs.feederVelocity = 30.0.rotations.perSecond
    assertFalse(feeder.isAtTargetedVelocity)
  }
}

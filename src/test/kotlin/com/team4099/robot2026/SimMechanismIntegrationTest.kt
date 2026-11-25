package com.team4099.robot2026

import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIO
import com.team4099.robot2026.subsystems.superstructure.feeder.FeederIOSim
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIO
import com.team4099.robot2026.subsystems.superstructure.hopper.HopperIOSim
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIO
import com.team4099.robot2026.subsystems.superstructure.shooter.ShooterIOSim
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIO
import com.team4099.robot2026.subsystems.superstructure.intake.IntakeIOSim
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIO
import com.team4099.robot2026.subsystems.superstructure.intake.rollers.IntakeRollersIOSim
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.perSecond

class SimMechanismIntegrationTest {
  @Test
  fun `velocity requests spin simulated mechanisms`() {
    System.setProperty("robot", "compbot")
    System.setProperty("tuning", "f")
    org.wpilib.hardware.hal.HAL.initialize()
    val feeder = FeederIO.FeederIOInputs()
    val hopper = HopperIO.HopperIOInputs()
    val shooter = ShooterIO.ShooterInputs()
    repeat(150) {
      FeederIOSim.setVelocity(100.0.radians.perSecond)
      HopperIOSim.setVelocity(50.0.radians.perSecond)
      ShooterIOSim.setVelocity(200.0.radians.perSecond)
      FeederIOSim.updateInputs(feeder)
      HopperIOSim.updateInputs(hopper)
      ShooterIOSim.updateInputs(shooter)
    }
    assertTrue(kotlin.math.abs(feeder.feederVelocity.value - 100.0) < 10.0, "Feeder: ${feeder.feederVelocity.value}")
    assertTrue(kotlin.math.abs(hopper.hopperAngularVelocity.value - 50.0) < 10.0, "Hopper: ${hopper.hopperAngularVelocity.value}")
    assertTrue(kotlin.math.abs(shooter.shooterLeaderVelocity.value - 200.0) < 20.0, "Shooter: ${shooter.shooterLeaderVelocity.value}")
    repeat(150) {
      FeederIOSim.setVelocity((-100.0).radians.perSecond)
      HopperIOSim.setVelocity((-50.0).radians.perSecond)
      ShooterIOSim.setVelocity(0.0.radians.perSecond)
      FeederIOSim.updateInputs(feeder)
      HopperIOSim.updateInputs(hopper)
      ShooterIOSim.updateInputs(shooter)
    }
    assertTrue(kotlin.math.abs(feeder.feederVelocity.value + 100.0) < 10.0, "Reverse feeder: ${feeder.feederVelocity.value}")
    assertTrue(kotlin.math.abs(hopper.hopperAngularVelocity.value + 50.0) < 10.0, "Reverse hopper: ${hopper.hopperAngularVelocity.value}")
    assertTrue(kotlin.math.abs(shooter.shooterLeaderVelocity.value) < 10.0, "Shooter did not stop: ${shooter.shooterLeaderVelocity.value}")
    val arm = IntakeIO.IntakeIOInputs()
    val rollers = IntakeRollersIO.RollerInputs()
    repeat(200) {
      IntakeIOSim.setPosition(30.0.degrees)
      IntakeIOSim.updateInputs(arm)
      IntakeRollersIOSim.setVoltage(6.0.volts)
      IntakeRollersIOSim.updateInputs(rollers)
    }
    assertTrue(kotlin.math.abs(arm.position.value - Math.PI / 6) < 0.15, "Intake arm: ${arm.position.value}")
    assertTrue(rollers.leaderVelocity.value > 1.0, "Intake rollers: ${rollers.leaderVelocity.value}")
  }
}

package com.team4099.robot2026
import com.team4099.robot2026.subsystems.drivetrain.ModuleIO
import com.team4099.robot2026.subsystems.drivetrain.ModuleIOSim
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertTrue
import org.wpilib.hardware.hal.HAL
import org.wpilib.math.geometry.Rotation2d
class SimSteeringTest {
  @Test fun `wheel turns and holds target angle`() {
    System.setProperty("robot", "compbot")
    System.setProperty("tuning", "f")
    HAL.initialize()
    val module = ModuleIOSim.generateModules()[0]
    val inputs = ModuleIO.ModuleIOInputs()
    module.setTurnPosition(Rotation2d.fromDegrees(90.0))
    repeat(150) { module.updateInputs(inputs) }
    val angle = inputs.turnPosition.value
    assertTrue(kotlin.math.abs(angle - Math.PI / 2) < 0.25, "Turn position $angle")
  }
}

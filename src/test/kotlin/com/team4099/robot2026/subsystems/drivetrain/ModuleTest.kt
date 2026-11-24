package com.team4099.robot2026.subsystems.drivetrain

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import kotlin.math.cos
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModuleVelocity

class ModuleTest {
  private class RecordingIO : ModuleIO {
    var commandedDriveVelocity = 0.0
    var commandedTurnPosition = Rotation2d.ZERO

    override fun setDriveVelocity(velocity: Double) {
      commandedDriveVelocity = velocity
    }

    override fun setTurnPosition(rotation: Rotation2d) {
      commandedTurnPosition = rotation
    }
  }

  @Test
  fun `setpoints apply the optimized and cosine scaled values`() {
    val io = RecordingIO()
    val constants =
        SwerveModuleConstants<
                TalonFXConfiguration?,
                TalonFXConfiguration?,
                CANcoderConfiguration?,
            >()
            .withWheelRadius(0.05)
    val module = Module(io, 0, constants)
    val requested = SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(150.0))

    module.runSetpoint(requested)

    assertEquals(-30.0, io.commandedTurnPosition.degrees, 1e-9)
    assertEquals(-2.0 * cos(Math.toRadians(30.0)) / 0.05, io.commandedDriveVelocity, 1e-9)
    assertEquals(2.0, requested.velocity, 1e-9)
    assertEquals(150.0, requested.angle.degrees, 1e-9)
  }

  @Test
  fun `wheel pointing uses the current module angle`() {
    val io = RecordingIO()
    val constants =
        SwerveModuleConstants<
            TalonFXConfiguration?,
            TalonFXConfiguration?,
            CANcoderConfiguration?,
        >()
    val module = Module(io, 1, constants)

    module.pointWheelsAt(org.team4099.lib.units.derived.Angle(Math.toRadians(170.0)))

    assertEquals(-10.0, io.commandedTurnPosition.degrees, 1e-9)
  }
}

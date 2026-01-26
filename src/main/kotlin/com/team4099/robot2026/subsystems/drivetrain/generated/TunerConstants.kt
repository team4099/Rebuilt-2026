package com.team4099.robot2026.subsystems.drivetrain.generated

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.team4099.lib.phoenix6.ConfiguredSwerveModuleConstants
import org.team4099.lib.units.LinearVelocity

sealed interface TunerConstants {
  val kCANBus: CANBus
  val kSpeedAt12Volts: LinearVelocity

  val CTREDrivetrainConstants: SwerveDrivetrainConstants

  val FrontLeft: ConfiguredSwerveModuleConstants
  val FrontRight: ConfiguredSwerveModuleConstants
  val BackLeft: ConfiguredSwerveModuleConstants
  val BackRight: ConfiguredSwerveModuleConstants
}

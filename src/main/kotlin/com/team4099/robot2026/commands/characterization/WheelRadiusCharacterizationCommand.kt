package com.team4099.robot2026.commands.characterization
// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import java.text.DecimalFormat
import java.text.NumberFormat
import kotlin.math.abs
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inRadiansPerSecond
import org.team4099.lib.units.inRadiansPerSecondPerSecond
import org.team4099.lib.units.perSecond

class WheelRadiusCharacterizationCommand(drivetrain: Drive) : ParallelCommandGroup() {
  private val limiter: SlewRateLimiter =
      SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE.inRadiansPerSecondPerSecond)
  private val state: WheelRadiusCharacterizationState = WheelRadiusCharacterizationState()

  init {
    addRequirements(drivetrain)

    addCommands(
        Commands.sequence(
            Commands.runOnce({ limiter.reset(0.0) }),
            Commands.run(
                {
                  drivetrain.runSpeeds(
                      ChassisSpeeds(
                          0.meters.perSecond,
                          0.meters.perSecond,
                          limiter
                              .calculate(WHEEL_RADIUS_MAX_VELOCITY.inRadiansPerSecond)
                              .radians
                              .perSecond))
                },
                drivetrain),
        ),
        Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce({
              state.positions = drivetrain.wheelRadiusCharacterizationPositions
              state.lastAngle = drivetrain.rotation.z.inRotation2ds
              state.gyroDelta = 0.0
            }),
            Commands.run({
                  val rotation = drivetrain.rotation.z.inRotation2ds
                  state.gyroDelta += abs(rotation.minus(state.lastAngle).radians)
                  state.lastAngle = rotation
                })
                .finallyDo(
                    Runnable {
                      val positions: DoubleArray = drivetrain.wheelRadiusCharacterizationPositions
                      var wheelDelta = 0.0
                      for (i in 0..3) {
                        wheelDelta += abs(positions[i] - state.positions[i]) / 4.0
                      }
                      val wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta

                      val formatter: NumberFormat = DecimalFormat("#0.000")
                      println("********** Wheel Radius Characterization Results **********")
                      println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians")
                      println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians")
                      println(
                          "\tWheel Radius: " +
                              formatter.format(wheelRadius) +
                              " meters, " +
                              formatter.format(wheelRadius.meters.inInches) +
                              " inches")

                      CustomLogger.recordOutput(
                          "WheelRadiusCharacterizationCommand/wheelDeltaRadians",
                          formatter.format(wheelDelta))
                      CustomLogger.recordOutput(
                          "WheelRadiusCharacterizationCommand/gyroDeltaRadians",
                          formatter.format(state.gyroDelta))
                      CustomLogger.recordOutput(
                          "WheelRadiusCharacterizationCommand/wheelRadiusInches",
                          formatter.format(wheelRadius.meters.inInches))
                    })))
  }

  private class WheelRadiusCharacterizationState {
    var positions: DoubleArray = DoubleArray(4)
    var lastAngle: Rotation2d = Rotation2d.kZero
    var gyroDelta: Double = 0.0
  }

  companion object {
    val WHEEL_RADIUS_RAMP_RATE = 0.05.radians.perSecond.perSecond
    val WHEEL_RADIUS_MAX_VELOCITY = 0.25.radians.perSecond
  }
}

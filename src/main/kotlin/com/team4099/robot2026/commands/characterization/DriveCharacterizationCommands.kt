package com.team4099.robot2026.commands.characterization

import com.team4099.robot2026.subsystems.drivetrain.Drive
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import java.text.DecimalFormat
import java.text.NumberFormat
import java.util.LinkedList
import kotlin.math.abs
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.perSecond

object DriveCharacterizationCommands {
  private const val FF_START_DELAY = 2.0
  private const val FF_RAMP_RATE = 0.1
  private const val WHEEL_RADIUS_MAX_VELOCITY = 2.0
  private const val WHEEL_RADIUS_RAMP_RATE = 0.15

  fun feedforwardCharacterization(drive: Drive): Command {
    val velocitySamples = LinkedList<Double>()
    val voltageSamples = LinkedList<Double>()
    val timer = Timer()

    return Commands.sequence(
        Commands.runOnce({
          velocitySamples.clear()
          voltageSamples.clear()
        }),
        Commands.run({ drive.runCharacterization(0.0) }, drive).withTimeout(FF_START_DELAY),
        Commands.runOnce({ timer.restart() }),
        Commands.run(
                {
                  val voltage = timer.get() * FF_RAMP_RATE
                  drive.runCharacterization(voltage)
                  velocitySamples.add(drive.ffCharacterizationVelocity)
                  voltageSamples.add(voltage)
                },
                drive)
            .finallyDo(
                Runnable {
                  val n = velocitySamples.size
                  var sumX = 0.0
                  var sumY = 0.0
                  var sumXY = 0.0
                  var sumX2 = 0.0

                  for (i in 0 until n) {
                    val x = velocitySamples[i]
                    val y = voltageSamples[i]
                    sumX += x
                    sumY += y
                    sumXY += x * y
                    sumX2 += x * x
                  }

                  val kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX)
                  val kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX)

                  val formatter: NumberFormat = DecimalFormat("#0.00000")
                  println("********** Drive FF Characterization Results **********")
                  println("\tkS: ${formatter.format(kS)}")
                  println("\tkV: ${formatter.format(kV)}")
                  CustomLogger.recordOutput("DriveCharacterizationCommands/kS", kS)
                  CustomLogger.recordOutput("DriveCharacterizationCommands/kV", kV)
                }))
  }

  fun wheelRadiusCharacterization(drive: Drive): Command {
    val limiter = SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE)
    val state = WheelRadiusCharacterizationState()

    val returnCommand =
        ParallelCommandGroup(
            SequentialCommandGroup(
                Commands.runOnce({ limiter.reset(0.0) }),
                Commands.run(
                        {
                          val speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY)
                          CustomLogger.recordOutput(
                              "DriveCharacterizationCommands/wheelRadiusOmegaRadPS", speed)
                          drive.runSpeeds(
                              ChassisSpeeds(
                                  0.0.meters.perSecond,
                                  0.0.meters.perSecond,
                                  speed.radians.perSecond))
                        },
                        drive)
                    .withTimeout(10.0)),
            SequentialCommandGroup(
                Commands.waitSeconds(10.0),
                Commands.runOnce({
                  state.positions = drive.wheelRadiusCharacterizationPositions
                  state.lastAngle = drive.rotation.z.inRotation2ds
                  state.gyroDelta = 0.0
                }),
                Commands.run({
                      val rotation = drive.rotation.z.inRotation2ds
                      state.gyroDelta += abs(rotation.minus(state.lastAngle).radians)
                      state.lastAngle = rotation
                    })
                    .finallyDo(
                        Runnable {
                          val positions = drive.wheelRadiusCharacterizationPositions
                          var wheelDelta = 0.0
                          for (i in 0 until 4) {
                            wheelDelta += abs(positions[i] - state.positions[i]) / 4.0
                          }

                          val wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta

                          val formatter: NumberFormat = DecimalFormat("#0.000")
                          println("********** Wheel Radius Characterization Results **********")
                          println("\tWheel Delta: ${formatter.format(wheelDelta)} radians")
                          println("\tGyro Delta: ${formatter.format(state.gyroDelta)} radians")
                          println(
                              "\tWheel Radius: ${formatter.format(wheelRadius)} meters, " +
                                  "${formatter.format(Units.metersToInches(wheelRadius))} inches")

                          CustomLogger.recordOutput(
                              "DriveCharacterizationCommands/wheelDeltaRadians", wheelDelta)
                          CustomLogger.recordOutput(
                              "DriveCharacterizationCommands/gyroDeltaRadians", state.gyroDelta)
                          CustomLogger.recordOutput(
                              "DriveCharacterizationCommands/wheelRadiusInches",
                              wheelRadius.meters.inInches)
                        })))

    returnCommand.name = "DriveCharacterizationCommands.wheelRadiusCharacterization"
    return returnCommand
  }

  private class WheelRadiusCharacterizationState {
    var positions: DoubleArray = DoubleArray(4)
    var lastAngle: Rotation2d = Rotation2d.kZero
    var gyroDelta: Double = 0.0
  }
}

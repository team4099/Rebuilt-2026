package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond

class TeleopDriveCommand(
  val driver: DriverProfile,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val turn: () -> Double,
  val slowMode: () -> Boolean,
  val drivetrain: Drive,
  private val flipIfRed: Boolean = true
) : Command() {

  private val joystick = CommandXboxController(0)

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {}

  override fun execute() {
    if (DriverStation.isTeleop()) {
      val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
      val rotation = driver.rotationSpeedClampedSupplier(turn, slowMode)

      CustomLogger.recordOutput("ActiveCommands/speedXinMPS", speed.first.inMetersPerSecond)
      CustomLogger.recordOutput("ActiveCommands/speedYinMPS", speed.second.inMetersPerSecond)
      CustomLogger.recordOutput("ActiveCommands/rotationInDPS", rotation.inDegreesPerSecond)

      drivetrain.runSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          speed.first, speed.second, rotation, drivetrain.pose.rotation.z
        )
      )

      CustomLogger.recordDebugOutput("ActiveCommands/TeleopDriveCommand", true)
    }
  }
  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordDebugOutput("ActiveCommands/TeleopDriveCommand", false)
  }
}

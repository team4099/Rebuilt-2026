package com.team4099.robot2025.commands.drivetrain

import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.CustomLogger
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.derived.radians
import kotlin.math.PI

class ResetGyroYawCommand(val drivetrain: Drive, private val flipIfRed: Boolean = true) :
  Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    val angle = if (flipIfRed && AllianceFlipUtil.shouldFlip()) -PI.radians else 0.radians
    drivetrain.pose = Pose3d(drivetrain.pose.translation, Rotation3d(0.radians, 0.radians, angle))
    drivetrain.pointWheelsAt(angle)
  }

  override fun execute() {
    CustomLogger.recordDebugOutput("ActiveCommands/ResetGyroYawCommand", true)
  }

  override fun isFinished(): Boolean {
    return true
  }
}

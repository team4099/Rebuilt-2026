package com.team4099.robot2026.commands

import com.team4099.robot2026.subsystems.superstructure.Request
import com.team4099.robot2026.subsystems.superstructure.shooter.Shooter
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.AngularVelocity

class ShooterTestCommand(private val shooter: Shooter, private val targetVelocity: AngularVelocity) : Command() {
  override fun initialize() {
    shooter.currentRequest = Request.ShooterRequest.TargetVelocity(targetVelocity)
  }

  override fun execute() {}

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {}
}
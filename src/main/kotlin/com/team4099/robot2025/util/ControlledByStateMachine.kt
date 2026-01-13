package com.team4099.robot2025.util

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry

/**
 * This abstract class creates "fake" subsystems with a "fake" periodic ([onLoop]). These will have
 * their loops run by Superstructure during its periodic. This enables us to cleanly control the
 * order in which subsystem loops run.
 *
 * Not all subsystems will be of this class. This class is only meant to be a substitute for classes
 * whose inputs are required to be the most recent for use in Superstructure. Some, like vision,
 * will remain in the old paradigm and be controlled by the `CommandScheduler`.
 *
 * @see [edu.wpi.first.wpilibj2.command.CommandScheduler]
 * @see [edu.wpi.first.wpilibj2.command.SubsystemBase]
 *
 * @param name Name of the class. If not passed in, the simple name of the class is used.
 * @author Nathan Areguzz, Ryan Chunguzz
 */
abstract class ControlledByStateMachine(name: String?) : Sendable {
  constructor() : this(null)

  init {
    val sentName = name ?: this.javaClass.simpleName
    SendableRegistry.addLW(this, sentName, sentName)
  }

  /**
   * The method to be run on every loop cycle, preceding the superstructure's state machine logic.
   */
  abstract fun onLoop()

  /**
   * @see [edu.wpi.first.wpilibj2.command.SubsystemBase]
   * @see [edu.wpi.first.util.sendable.Sendable]
   */
  final override fun initSendable(builder: SendableBuilder) {
    builder.setSmartDashboardType("Subsystem")
  }
}

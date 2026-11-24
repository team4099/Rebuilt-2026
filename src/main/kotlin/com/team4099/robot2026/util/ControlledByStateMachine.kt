package com.team4099.robot2026.util

/**
 * This abstract class creates "fake" subsystems with a "fake" periodic ([onLoop]). These will have
 * their loops run by Superstructure during its periodic. This enables us to cleanly control the
 * order in which subsystem loops run.
 *
 * Not all subsystems will be of this class. This class is only meant to be a substitute for classes
 * whose inputs are required to be the most recent for use in Superstructure. Some, like vision,
 * will remain in the old paradigm and be controlled by the `CommandScheduler`.
 *
 * @param name Name of the class. If not passed in, the simple name of the class is used.
 * @see [org.wpilib.command2.CommandScheduler]
 * @see [org.wpilib.command2.SubsystemBase]
 * @author Nathan Areguzz, Ryan Chunguzz
 */
abstract class ControlledByStateMachine(name: String?) {
  constructor() : this(null)

  /**
   * The method to be run on every loop cycle, preceding the superstructure's state machine logic.
   */
  abstract fun onLoop()
}

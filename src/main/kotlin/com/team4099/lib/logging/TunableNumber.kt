package com.team4099.lib.logging

import com.team4099.robot2026.config.constants.Constants
import org.wpilib.backend.NetworkTablesTunableBackend
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.tunable.TunableDouble
import org.wpilib.tunable.TunableRegistry
import org.wpilib.tunable.Tunables

/**
 * Taken from
 * https://github.com/Mechanical-Advantage/SwerveDevelopment/blob/main/src/main/java/frc/robot/util/TunableNumber.java
 */
class TunableNumber(dashboardKey: String) {
  private val key: String
  private var tunable: TunableDouble? = null
  private var defaultValue = 0.0
  private var lastHasChangedValue = defaultValue

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  constructor(dashboardKey: String, defaultValue: Double) : this(dashboardKey) {
    setDefault(defaultValue)
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  fun getDefault(): Double {
    return defaultValue
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  fun setDefault(defaultValue: Double) {
    this.defaultValue = defaultValue

    if (tunable == null) tunable = Tunables.addDouble(key, defaultValue)
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  fun get(): Double {
    return if (Constants.Tuning.TUNING_MODE) tunable?.get() ?: defaultValue else defaultValue
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *   otherwise
   */
  fun hasChanged(): Boolean {
    val currentValue = get()
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue
      return true
    }
    return false
  }

  companion object {
    private const val tableKey = "/TunableNumbers"

    init {
      TunableRegistry.registerBackend(
          tableKey,
          NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), "/SmartDashboard"))
    }
  }

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  init {
    key = tableKey + "/" + dashboardKey
  }
}

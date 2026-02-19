// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package com.team4099.lib.logging

import com.team4099.robot2026.config.constants.Constants
import java.util.Arrays
import java.util.function.Consumer
import java.util.function.DoubleSupplier
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
@Suppress("unused")
class LoggedTunableNumber(dashboardKey: String?) : DoubleSupplier {
  private val key: String = "$tableKey/$dashboardKey"
  private var hasDefault = false
  private var defaultValue = 0.0
  private var dashboardNumber: LoggedNetworkNumber? = null
  private val lastHasChangedValues: MutableMap<Int?, Double?> = HashMap<Int?, Double?>()

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  constructor(dashboardKey: String?, defaultValue: Double) : this(dashboardKey) {
    initDefault(defaultValue)
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  fun initDefault(defaultValue: Double) {
    if (!hasDefault) {
      hasDefault = true
      this.defaultValue = defaultValue
      if (Constants.Tuning.TUNING_MODE) {
        dashboardNumber = LoggedNetworkNumber(key, defaultValue)
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  fun get(): Double {
    return if (!hasDefault) {
      0.0
    } else {
      if (Constants.Tuning.TUNING_MODE) dashboardNumber!!.get() else defaultValue
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *   objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *   otherwise.
   */
  fun hasChanged(id: Int): Boolean {
    val currentValue = get()
    val lastValue = lastHasChangedValues[id]
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues[id] = currentValue
      return true
    }

    return false
  }

  override fun getAsDouble(): Double {
    return get()
  }

  companion object {
    private const val tableKey = "/Tuning"

    /**
     * Runs action if any of the tunableNumbers have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
     *   objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable
     *   numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    fun ifChanged(
        id: Int,
        action: Consumer<DoubleArray?>,
        vararg tunableNumbers: LoggedTunableNumber?
    ) {
      if (Arrays.stream<LoggedTunableNumber?>(tunableNumbers).anyMatch {
          tunableNumber: LoggedTunableNumber? ->
        tunableNumber!!.hasChanged(id)
      }) {
        action.accept(
            Arrays.stream<LoggedTunableNumber?>(tunableNumbers)
                .mapToDouble { obj: LoggedTunableNumber? -> obj!!.get() }
                .toArray())
      }
    }

    /** Runs action if any of the tunableNumbers have changed */
    fun ifChanged(id: Int, action: Runnable, vararg tunableNumbers: LoggedTunableNumber?) {
      ifChanged(id, { values: DoubleArray? -> action.run() }, *tunableNumbers)
    }
  }
}

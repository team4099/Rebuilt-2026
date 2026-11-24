package com.team4099.lib.logging

import org.wpilib.networktables.NetworkTableInstance

/** Legacy dashboard topics shared with the existing driver layout. */
object Dashboard {
  private val table = NetworkTableInstance.getDefault().getTable("SmartDashboard")

  fun putNumber(key: String, value: Double) = table.getEntry(key).setDouble(value)

  fun getNumber(key: String, defaultValue: Double) = table.getEntry(key).getDouble(defaultValue)

  fun putBoolean(key: String, value: Boolean) = table.getEntry(key).setBoolean(value)

  fun getBoolean(key: String, defaultValue: Boolean) = table.getEntry(key).getBoolean(defaultValue)

  fun putString(key: String, value: String) = table.getEntry(key).setString(value)

  fun getString(key: String, defaultValue: String) = table.getEntry(key).getString(defaultValue)
}

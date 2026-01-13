// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package com.team4099.robot2025.subsystems.drivetrain

import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface ModuleIO {
  class ModuleIOInputs : LoggableInputs {
    var driveConnected: Boolean = false
    var drivePosition: Angle = 0.0.radians
    var driveVelocity: AngularVelocity = 0.0.radians.perSecond
    var driveAppliedVoltage: ElectricalPotential = 0.0.volts
    var driveCurrent: Current = 0.0.amps

    var turnConnected: Boolean = false
    var turnEncoderConnected: Boolean = false
    var turnAbsolutePosition: Angle = 0.0.radians
    var turnPosition: Angle = 0.0.radians
    var turnVelocity: AngularVelocity = 0.0.radians.perSecond
    var turnAppliedVoltage: ElectricalPotential = 0.0.volts
    var turnCurrent: Current = 0.0.amps

    var odometryTimestamps: DoubleArray = doubleArrayOf()
    var odometryDrivePositions: Array<Angle> = arrayOf()
    var odometryTurnPositions: Array<Angle> = arrayOf()

    override fun toLog(table: LogTable) {
      table.put("driveConnected", driveConnected)
      table.put("drivePositionDegrees", drivePosition.inDegrees)
      table.put("driveVelocityDegPerSec", driveVelocity.inDegreesPerSecond)
      table.put("driveAppliedVoltage", driveAppliedVoltage.inVolts)
      table.put("driveCurrent", driveCurrent.inAmperes)

      table.put("turnConnected", turnConnected)
      table.put("turnEncoderConnected", turnEncoderConnected)
      table.put("turnAbsolutePositionDegrees", turnAbsolutePosition.inDegrees)
      table.put("turnPositionDegrees", turnPosition.inDegrees)
      table.put("turnVelocityDegPerSec", turnVelocity.inDegreesPerSecond)
      table.put("turnAppliedVoltage", turnAppliedVoltage.inVolts)
      table.put("turnCurrent", turnCurrent.inAmperes)

      table.put("odometryTimestamps", odometryTimestamps)
      table.put(
        "odometryDrivePositionsDegrees",
        odometryDrivePositions.map { angle: Angle -> angle.inDegrees }.toDoubleArray()
      )
      table.put(
        "odometryTurnPositionsDegrees",
        odometryTurnPositions.map { angle: Angle -> angle.inDegrees }.toDoubleArray()
      )
    }

    override fun fromLog(table: LogTable) {
      table.get("driveConnected", driveConnected).let { driveConnected = it }
      table.get("drivePositionDegrees", drivePosition.inDegrees).let { drivePosition = it.degrees }
      table.get("driveVelocityDegPerSec", driveVelocity.inDegreesPerSecond).let {
        driveVelocity = it.degrees.perSecond
      }
      table.get("driveAppliedVoltage", driveAppliedVoltage.inVolts).let {
        driveAppliedVoltage = it.volts
      }
      table.get("driveCurrent", driveCurrent.inAmperes).let { driveCurrent = it.amps }

      table.get("turnConnected", turnConnected).let { turnConnected = it }
      table.get("turnEncoderConnected", turnEncoderConnected).let { turnEncoderConnected = it }
      table.get("turnAbsolutePositionDegrees", turnAbsolutePosition.inDegrees).let {
        turnAbsolutePosition = it.degrees
      }
      table.get("turnPositionDegrees", turnPosition.inDegrees).let { turnPosition = it.degrees }
      table.get("turnVelocityDegPerSec", turnVelocity.inDegreesPerSecond).let {
        turnVelocity = it.degrees.perSecond
      }
      table.get("turnAppliedVoltage", turnAppliedVoltage.inVolts).let {
        turnAppliedVoltage = it.volts
      }
      table.get("turnCurrent", turnCurrent.inAmperes).let { turnCurrent = it.amps }

      table.get("odometryTimestamps", odometryTimestamps).let { odometryTimestamps = it }
      table.get(
        "odometryDrivePositionsDegrees",
        odometryDrivePositions.map { angle: Angle -> angle.inDegrees }.toDoubleArray()
      )
        .let {
          odometryDrivePositions =
            it.map { angleDegrees: Double -> angleDegrees.degrees }.toTypedArray()
        }
      table.get(
        "odometryTurnPositionDegrees",
        odometryTurnPositions.map { angle: Angle -> angle.inDegrees }.toDoubleArray()
      )
        .let {
          odometryTurnPositions =
            it.map { angleDegrees: Double -> angleDegrees.degrees }.toTypedArray()
        }
    }
  }

  /** Updates the set of loggable inputs. */
  fun updateInputs(inputs: ModuleIOInputs) {}

  /** Run the drive motor at the specified open loop value. */
  fun setDriveOpenLoop(output: Double) {}

  /** Run the turn motor at the specified open loop value. */
  fun setTurnOpenLoop(output: Double) {}

  /** Run the drive motor at the specified velocity. */
  fun setDriveVelocity(velocityRadPerSec: Double) {}

  /** Run the turn motor to the specified rotation. */
  fun setTurnPosition(rotation: Rotation2d) {}

  /** Enable/disable brake mode on the drive and steer motors */
  fun toggleBrakeMode(brake: NeutralModeValue)
}

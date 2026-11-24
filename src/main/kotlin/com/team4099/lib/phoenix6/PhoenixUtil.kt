// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
// Also taken from AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced
// Available at
// https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced

package com.team4099.lib.phoenix6

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import java.util.function.Supplier

object PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
    for (i in 0 until maxAttempts) {
      val error = command.get()
      if (error.isOK) break
    }
  }

  /** Signals for synchronized refresh. */
  private var canivoreSignals = arrayOfNulls<BaseStatusSignal>(0)

  private var rioSignals = arrayOfNulls<BaseStatusSignal>(0)

  /** Registers a set of signals for synchronized refresh. */
  fun registerSignals(canivore: Boolean, vararg signals: BaseStatusSignal?) {
    if (canivore) {
      val newSignals = arrayOfNulls<BaseStatusSignal>(canivoreSignals.size + signals.size)
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.size)
      System.arraycopy(signals, 0, newSignals, canivoreSignals.size, signals.size)
      canivoreSignals = newSignals
    } else {
      val newSignals = arrayOfNulls<BaseStatusSignal>(rioSignals.size + signals.size)
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.size)
      System.arraycopy(signals, 0, newSignals, rioSignals.size, signals.size)
      rioSignals = newSignals
    }
  }

  /** Refresh all registered signals. */
  fun refreshAll() {
    if (canivoreSignals.size > 0) {
      BaseStatusSignal.refreshAll(*canivoreSignals)
    }
    if (rioSignals.size > 0) {
      BaseStatusSignal.refreshAll(*rioSignals)
    }
  }
}

typealias ConfiguredSwerveModuleConstants =
    SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>

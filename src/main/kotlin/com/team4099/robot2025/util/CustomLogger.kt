package com.team4099.robot2025.util

import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.util.WPISerializable
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs

class CustomLogger {
  companion object {
    inline fun processInputs(key: String, inputs: LoggableInputs) {
      Logger.processInputs(key, inputs)
    }

    inline fun <E : Enum<E>> recordOutput(key: String, value: E) {
      Logger.recordOutput(key, value)
    }

    inline fun <T : WPISerializable> recordOutput(key: String, value: T) {
      Logger.recordOutput(key, value)
    }

    inline fun <T> recordOutput(key: String, value: Struct<T>) {
      Logger.recordOutput(key, value)
    }

    inline fun <T : StructSerializable> recordOutput(key: String, vararg value: T) {
      Logger.recordOutput(key, *value)
    }

    inline fun recordOutput(key: String, value: Array<String>) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: BooleanArray) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: Boolean) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: ByteArray) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: DoubleArray) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: Double) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: Float) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: FloatArray) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: Int) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: IntArray) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: Long) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: LongArray) {
      Logger.recordOutput(key, value)
    }

    inline fun recordOutput(key: String, value: String) {
      Logger.recordOutput(key, value)
    }

    inline fun <E : Enum<E>> recordDebugOutput(key: String, value: E) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun <T : WPISerializable> recordDebugOutput(key: String, value: T) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun <T> recordDebugOutput(key: String, value: Struct<T>) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun <T : StructSerializable> recordDebugOutput(key: String, vararg value: T) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, *value)
      }
    }

    inline fun recordDebugOutput(key: String, value: Array<String>) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: BooleanArray) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: Boolean) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: ByteArray) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: DoubleArray) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: Double) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: Float) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: FloatArray) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: Int) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: IntArray) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: Long) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: LongArray) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }

    inline fun recordDebugOutput(key: String, value: String) {
      if (Constants.Tuning.DEBUGING_MODE) {
        Logger.recordOutput(key, value)
      }
    }
  }
}

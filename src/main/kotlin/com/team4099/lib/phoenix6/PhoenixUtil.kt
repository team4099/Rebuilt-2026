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
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.sim.CANcoderSimState
import com.ctre.phoenix6.sim.TalonFXSimState
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.motorsims.SimulatedBattery
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerMeterPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerMeters
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecond
import org.team4099.lib.units.derived.inVoltsPerMetersPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianSeconds
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

  val simulationOdometryTimeStamps: DoubleArray
    get() {
      val odometryTimeStamps = DoubleArray(SimulatedArena.getSimulationSubTicksIn1Period())
      for (i in odometryTimeStamps.indices) {
        odometryTimeStamps[i] =
          (
            Timer.getFPGATimestamp() - 0.02 +
              i * SimulatedArena.getSimulationDt().`in`(Units.Seconds)
            )
      }

      return odometryTimeStamps
    }

  fun regulateModuleConstantForSimulation(
    moduleConstants: ConfiguredSwerveModuleConstants
  ): ConfiguredSwerveModuleConstants {
    if (RobotBase.isReal()) {
      return moduleConstants
    }

    return moduleConstants
      .withEncoderOffset(0.0)
      .withDriveMotorInverted(false)
      .withSteerMotorInverted(false)
      .withEncoderInverted(false)
      .withDriveMotorGains(
        Slot0Configs()
          .withKP(DrivetrainConstants.PID.SIM_DRIVE_KP.inVoltsPerMetersPerSecond)
          .withKI(DrivetrainConstants.PID.SIM_DRIVE_KI.inVoltsPerMeters)
          .withKD(DrivetrainConstants.PID.SIM_DRIVE_KD.inVoltsPerMetersPerSecondPerSecond)
          .withKS(DrivetrainConstants.PID.SIM_DRIVE_KS.inVolts)
          .withKV(DrivetrainConstants.PID.SIM_DRIVE_KV.inVoltsPerMetersPerSecond)
          .withKA(DrivetrainConstants.PID.SIM_DRIVE_KA.inVoltsPerMeterPerSecondPerSecond)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
      )
      .withSteerMotorGains(
        Slot0Configs()
          .withKP(DrivetrainConstants.PID.SIM_STEERING_KP.inVoltsPerRadian)
          .withKI(DrivetrainConstants.PID.SIM_STEERING_KI.inVoltsPerRadianSeconds)
          .withKD(DrivetrainConstants.PID.SIM_STEERING_KD.inVoltsPerRadianPerSecond)
          .withKS(0.0)
          .withKV(DrivetrainConstants.PID.SIM_STEERING_KV.inVoltsPerRadianPerSecond)
          .withKA(0.0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
      )
      .withSlipCurrent(120.0)
  }

  open class TalonFXMotorControllerSim(talonFX: TalonFX) : SimulatedMotorController {
    val id: Int

    private val talonFXSimState: TalonFXSimState

    init {
      this.id = instances++

      this.talonFXSimState = talonFX.simState
    }

    override fun updateControlSignal(
      mechanismAngle: Angle,
      mechanismVelocity: AngularVelocity,
      encoderAngle: Angle,
      encoderVelocity: AngularVelocity
    ): Voltage {
      talonFXSimState.setRawRotorPosition(encoderAngle)
      talonFXSimState.setRotorVelocity(encoderVelocity)
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage())
      return talonFXSimState.motorVoltageMeasure
    }

    companion object {
      private var instances = 0
    }
  }

  class TalonFXMotorControllerWithRemoteCancoderSim(talonFX: TalonFX, cancoder: CANcoder) :
    TalonFXMotorControllerSim(talonFX) {
    private val remoteCancoderSimState: CANcoderSimState = cancoder.simState

    override fun updateControlSignal(
      mechanismAngle: Angle,
      mechanismVelocity: AngularVelocity,
      encoderAngle: Angle,
      encoderVelocity: AngularVelocity
    ): Voltage {
      remoteCancoderSimState.setRawPosition(mechanismAngle)
      remoteCancoderSimState.setVelocity(mechanismVelocity)

      return super.updateControlSignal(
        mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity
      )
    }
  }
}

private typealias ConfiguredSwerveModuleConstants =
  SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>

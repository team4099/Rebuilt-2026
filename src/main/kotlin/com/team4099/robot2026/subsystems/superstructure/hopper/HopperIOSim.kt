package com.team4099.robot2026.subsystems.superstructure.hopper

import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.HopperConstants
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.math.clamp
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Second
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.FlywheelSim

object HopperIOSim : HopperIO {
  private var appliedVoltage = 0.0.volts
  private val hopperSim =
      FlywheelSim(
          Models.flywheelFromPhysicalConstants(
              DCMotor.getKrakenX44Foc(1),
              HopperConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / HopperConstants.GEAR_RATIO,
          ),
          DCMotor.getKrakenX44Foc(1),
      )

  private val hopperPIDController =
      PIDController(
          HopperConstants.PID.SIM_KP, HopperConstants.PID.SIM_KI, HopperConstants.PID.SIM_KD)

  private var hopperFFController =
      SimpleMotorFeedforward(
          HopperConstants.PID.SIM_KS, HopperConstants.PID.SIM_KV, HopperConstants.PID.SIM_KA)

  override fun updateInputs(inputs: HopperIO.HopperIOInputs) {
    hopperSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.hopperAngularVelocity = hopperSim.angularVelocity.radians.perSecond
    inputs.hopperAngularAcceleration = hopperSim.angularAcceleration.radians.perSecond.perSecond
    inputs.hopperAppliedVoltage = appliedVoltage
    inputs.hopperSupplyCurrent = 0.0.amps
    inputs.hopperStatorCurrent = hopperSim.currentDraw.amps
    inputs.hopperTemp = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -HopperConstants.VOLTAGE_COMPENSATION, HopperConstants.VOLTAGE_COMPENSATION)

    hopperSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setVelocity(velocity: AngularVelocity) {
    var pidOutput =
        hopperPIDController.calculate(hopperSim.angularVelocity.radians.perSecond, velocity)
    if (pidOutput.inVolts.isNaN()) pidOutput = 0.volts
    val ffOutput =
        hopperFFController.calculate(hopperSim.angularVelocity.radians.perSecond, velocity)
    setVoltage(pidOutput + ffOutput)
  }

  override fun configurePIDVoltage(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    hopperPIDController.setPID(kP, kI, kD)
  }

  override fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    hopperFFController = SimpleMotorFeedforward(kS, kV, kA)
  }
}

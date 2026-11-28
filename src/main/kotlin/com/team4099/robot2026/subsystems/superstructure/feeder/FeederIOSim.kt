package com.team4099.robot2026.subsystems.superstructure.feeder

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FeederConstants
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.FlywheelSim

object FeederIOSim : FeederIO {
  private var appliedVoltage = 0.0.volts
  private val subsystemSim =
      FlywheelSim(
          Models.flywheelFromPhysicalConstants(
              DCMotor.getKrakenX44(1),
              FeederConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1 / FeederConstants.GEAR_RATIO),
          DCMotor.getKrakenX44(1))

  private val feederPIDController =
      PIDController(
          FeederConstants.PID.SIM_KP, FeederConstants.PID.SIM_KI, FeederConstants.PID.SIM_KD)

  private var feederFFController =
      SimpleMotorFeedforward(
          FeederConstants.PID.SIM_KS, FeederConstants.PID.SIM_KV, FeederConstants.PID.SIM_KA)

  override fun updateInputs(inputs: FeederIO.FeederIOInputs) {
    subsystemSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.feederVelocity = subsystemSim.angularVelocity.radians.perSecond
    inputs.feederAppliedVoltage = appliedVoltage
    inputs.feederSupplyCurrent = subsystemSim.currentDraw.amps
    inputs.feederStatorCurrent = 0.0.amps
    inputs.feederTemp = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -FeederConstants.VOLTAGE_COMPENSATION, FeederConstants.VOLTAGE_COMPENSATION)
    subsystemSim.setInputVoltage(clampedVoltage.inVolts)
    appliedVoltage = clampedVoltage
  }

  override fun setVelocity(velocity: AngularVelocity) {
    var pidOutput =
        feederPIDController.calculate(subsystemSim.angularVelocity.radians.perSecond, velocity)
    if (pidOutput.inVolts.isNaN()) pidOutput = 0.volts
    val ffOutput =
        feederFFController.calculate(subsystemSim.angularVelocity.radians.perSecond, velocity)
    setVoltage(pidOutput + ffOutput)
  }
}

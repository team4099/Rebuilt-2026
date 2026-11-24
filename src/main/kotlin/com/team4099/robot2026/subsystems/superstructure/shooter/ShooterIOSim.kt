package com.team4099.robot2026.subsystems.superstructure.shooter

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.ShooterConstants
import org.team4099.lib.controller.PIDController
import org.team4099.lib.controller.SimpleMotorFeedforward
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

object ShooterIOSim : ShooterIO {
  private val shooterSim: FlywheelSim =
      FlywheelSim(
          Models.flywheelFromPhysicalConstants(
              DCMotor.getKrakenX60(2),
              ShooterConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1.0 / ShooterConstants.GEAR_RATIO,
          ),
          DCMotor.getKrakenX60(2))

  private val shooterPIDController =
      PIDController(
          ShooterConstants.PID.SIM_KP, ShooterConstants.PID.SIM_KI, ShooterConstants.PID.SIM_KD)

  private var shooterFFController =
      SimpleMotorFeedforward(
          ShooterConstants.PID.SIM_KS, ShooterConstants.PID.SIM_KV, ShooterConstants.PID.SIM_KA)

  override fun updateInputs(inputs: ShooterIO.ShooterInputs) {
    shooterSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.shooterLeaderVelocity = shooterSim.angularVelocity.radians.perSecond
    inputs.shooterLeaderAcceleration = shooterSim.angularAcceleration.radians.perSecond.perSecond
    inputs.shooterLeaderVoltage = shooterSim.inputVoltage.volts
    inputs.shooterLeaderSupplyCurrent = 0.0.amps
    inputs.shooterLeaderStatorCurrent = shooterSim.currentDraw.amps
    inputs.shooterLeaderTorqueCurrent = shooterSim.currentDraw.amps.absoluteValue
    inputs.shooterLeaderTemperature = 0.0.celsius

    inputs.shooterFollowerVelocity = shooterSim.angularVelocity.radians.perSecond
    inputs.shooterFollowerAcceleration = shooterSim.angularAcceleration.radians.perSecond.perSecond
    inputs.shooterFollowerVoltage = shooterSim.inputVoltage.volts
    inputs.shooterFollowerSupplyCurrent = 0.0.amps
    inputs.shooterFollowerStatorCurrent = shooterSim.currentDraw.amps
    inputs.shooterFollowerTorqueCurrent = shooterSim.currentDraw.amps.absoluteValue
    inputs.shooterFollowerTemperature = 0.0.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage, -ShooterConstants.VOLTAGE_COMPENSATION, ShooterConstants.VOLTAGE_COMPENSATION)
    shooterSim.setInputVoltage(clampedVoltage.inVolts)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    var pidOutput =
        shooterPIDController.calculate(shooterSim.angularVelocity.radians.perSecond, velocity)
    if (pidOutput.inVolts.isNaN()) pidOutput = 0.volts
    val ffOutput =
        shooterFFController.calculate(shooterSim.angularVelocity.radians.perSecond, velocity)
    setVoltage(pidOutput + ffOutput)
  }

  override fun configurePIDVoltage(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    shooterPIDController.setPID(kP, kI, kD)
  }

  override fun configureFFVoltage(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    shooterFFController = SimpleMotorFeedforward(kS, kV, kA)
  }
}

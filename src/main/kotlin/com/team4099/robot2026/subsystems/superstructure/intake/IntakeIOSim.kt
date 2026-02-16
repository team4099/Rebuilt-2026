package com.team4099.robot2026.subsystems.superstructure.intake

import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.IntakeConstants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team4099.lib.controller.ProfiledPIDController
import org.team4099.lib.controller.TrapezoidProfile
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeIOSim : IntakeIO {

  private val armSim: SingleJointedArmSim =
      SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          1.0 / IntakeConstants.GEAR_RATIO,
          IntakeConstants.PIVOT_INERTIA.inKilogramsMeterSquared,
          IntakeConstants.PIVOT_LENGTH.inMeters,
          IntakeConstants.PIVOT_MIN_ANGLE.inRadians,
          IntakeConstants.PIVOT_MAX_ANGLE.inRadians,
          false,
          0.0)

  private val armPIDController =
      ProfiledPIDController(
          IntakeConstants.PID.SIM_PIVOT_KP,
          IntakeConstants.PID.SIM_PIVOT_KI,
          IntakeConstants.PID.SIM_PIVOT_KD,
          TrapezoidProfile.Constraints(
              IntakeConstants.SIM_VELOCITY, IntakeConstants.SIM_ACCELERATION))

  private var pivotAppliedVoltage = 0.0.volts

  init {
    armSim.setState(0.0, 0.0)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    armSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.position = armSim.angleRads.radians
    inputs.velocity = armSim.velocityRadPerSec.radians.perSecond

    inputs.intakeAppliedVoltage = pivotAppliedVoltage
    inputs.intakeStatorCurrent = armSim.currentDrawAmps.amps
    inputs.intakeSupplyCurrent = 0.amps
    inputs.intakeTemperature = 0.0.celsius

    inputs.isSimulating = true
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
    armSim.setInputVoltage(clampedVoltage.inVolts)
    pivotAppliedVoltage = clampedVoltage
  }

  override fun setPosition(position: Angle) {
    val feedback = armPIDController.calculate(armSim.angleRads.radians, position)
    setVoltage(feedback)
  }

  override fun zeroPivot() {}

  override fun configPID(
      kPUp: ProportionalGain<Radian, Volt>,
      kPDown: ProportionalGain<Radian, Volt>,
      kI: IntegralGain<Radian, Volt>,
      kD: DerivativeGain<Radian, Volt>
  ) {
    armPIDController.setPID(kPDown, kI, kD)
  }
}

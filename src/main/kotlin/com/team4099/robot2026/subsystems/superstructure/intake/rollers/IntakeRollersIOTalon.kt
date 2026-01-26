package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.RollersConstants
import com.team4099.robot2026.subsystems.superstructure.Intake.Rollers.IntakeRollersIO
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object IntakeRollersIOTalon : IntakeRollersIO {

  private val rollerTalon: TalonFX = TalonFX(Constants.Intake.INTAKE_ROLLERS_MOTOR_ID)

  private val rollerConfig: TalonFXConfiguration = TalonFXConfiguration()

  var statorCurrent: StatusSignal<Current>
  var supplyCurrent: StatusSignal<Current>
  var tempSignal: StatusSignal<Temperature>
  var motorVoltage: StatusSignal<Voltage>
  var motorAccel: StatusSignal<AngularAcceleration>
  var motorVelo: StatusSignal<AngularVelocity>

  val voltageControl = VoltageOut(0.0.volts.inVolts)

  private var rollerSensor =
      ctreAngularMechanismSensor(
          rollerTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION)

  init {
    rollerTalon.clearStickyFaults()

    rollerConfig.CurrentLimits.SupplyCurrentLimit = RollersConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    rollerConfig.CurrentLimits.StatorCurrentLimit = RollersConstants.STATOR_CURRENT_LIMIT.inAmperes
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    statorCurrent = rollerTalon.statorCurrent
    supplyCurrent = rollerTalon.supplyCurrent
    tempSignal = rollerTalon.deviceTemp
    motorVoltage = rollerTalon.motorVoltage
    motorAccel = rollerTalon.acceleration
    motorVelo = rollerTalon.velocity

    rollerTalon.configurator.apply(rollerConfig)
  }

  override fun updateInputs(inputs: IntakeRollersIO.RollerInputs) {
    refreshStatusSignals()
    inputs.rollerVelocity = rollerSensor.velocity
    inputs.rollerAppliedVoltage = motorVoltage.valueAsDouble.volts
    inputs.rollerStatorCurrent = statorCurrent.valueAsDouble.amps
    inputs.rollerSupplyCurrent = supplyCurrent.valueAsDouble.amps
    inputs.rollerTemperature = tempSignal.valueAsDouble.celsius
    inputs.rollerAcceleration =
        motorAccel.valueAsDouble.rotations.perSecond.perSecond * RollersConstants.GEAR_RATIO
  }

  fun refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        supplyCurrent, statorCurrent, motorVoltage, motorAccel, tempSignal, motorVelo)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage, -RollersConstants.VOLTAGE_COMPENSATION, RollersConstants.VOLTAGE_COMPENSATION)
    rollerTalon.setControl(voltageControl.withOutput(clampedVoltage.inVolts))
  }
}

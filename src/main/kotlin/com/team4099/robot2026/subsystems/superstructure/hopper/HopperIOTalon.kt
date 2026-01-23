package com.team4099.robot2026.subsystems.superstructure.hopper

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.HopperConstants
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature as WPITemp
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

object HopperIOTalon : HopperIO {

  private val hopperTalon: TalonFX = TalonFX(Constants.Hopper.HOPPER_MOTOR_ID)
  private val voltageOut = VoltageOut(-1337.volts.inVolts)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val hopperSensor =
      ctreAngularMechanismSensor(
          hopperTalon, HopperConstants.GEAR_RATIO, HopperConstants.VOLTAGE_COMPENSATION)

  var statorCurrentSignal: StatusSignal<Current>
  var supplyCurrentSignal: StatusSignal<Voltage>
  var tempSignal: StatusSignal<WPITemp>
  var dutyCycleSignal: StatusSignal<Double>
  var motorVoltageSignal: StatusSignal<Voltage>
  var motorAccelSignal: StatusSignal<WPIAngularAcceleration>
  var rotorVelocitySignal: StatusSignal<WPIAngularVelocity>

  init {
    hopperTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = HopperConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = HopperConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast

    hopperTalon.configurator.apply(configs)

    rotorVelocitySignal = hopperTalon.velocity
    statorCurrentSignal = hopperTalon.supplyCurrent
    supplyCurrentSignal = hopperTalon.supplyVoltage
    tempSignal = hopperTalon.deviceTemp
    dutyCycleSignal = hopperTalon.dutyCycle
    motorVoltageSignal = hopperTalon.motorVoltage
    motorAccelSignal = hopperTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        statorCurrentSignal,
        supplyCurrentSignal,
        tempSignal,
        dutyCycleSignal,
        motorVoltageSignal,
        motorAccelSignal,
        rotorVelocitySignal)
  }

  override fun updateInputs(inputs: HopperIO.HopperIOInputs) {
    updateSignals()
    inputs.hopperAppliedVoltage = motorVoltageSignal.valueAsDouble.volts
    inputs.hopperStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.hopperSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.hopperTemp = tempSignal.valueAsDouble.celsius
    inputs.hopperAcceleration =
        (motorAccelSignal.valueAsDouble / HopperConstants.GEAR_RATIO).rotations.perSecond.perSecond
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    hopperTalon.setControl(voltageOut.withOutput(voltage.inVolts))
  }

  override fun setBrakeMode(brake: Boolean) {
    hopperTalon.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
  }
}

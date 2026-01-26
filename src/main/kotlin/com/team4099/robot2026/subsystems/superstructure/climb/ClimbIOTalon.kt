package com.team4099.robot2026.subsystems.superstructure.climb

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2026.config.constants.ClimbConstants
import com.team4099.robot2026.config.constants.ClimbConstants.MAX_ACCELERATION
import com.team4099.robot2026.config.constants.ClimbConstants.MAX_VELOCITY
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond

object ClimbIOTalon : ClimbIO {
  private val talon: TalonFX = TalonFX(Constants.Climb.MOTOR_ID)

  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs = configs.Slot0

  private val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)
  private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.0)

  private val sensor =
      ctreLinearMechanismSensor(
          talon,
          ClimbConstants.GEAR_RATIO,
          ClimbConstants.DRUM_DIAMETER,
          ClimbConstants.VOLTAGE_COMPENSATION)

  private var statorCurrentSignal: StatusSignal<WPILibCurrent>
  private var supplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var tempSignal: StatusSignal<WPILibTemperature>
  private var positionSignal: StatusSignal<Angle>
  private var velocitySignal: StatusSignal<AngularVelocity>

  private var motorVoltage: StatusSignal<WPILibVoltage>

  init {
    talon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLowerLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLowerTime = ClimbConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.CurrentLimits.SupplyCurrentLimitEnable = true

    configs.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.inInchesPerSecond
    configs.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.inInchesPerSecondPerSecond

    positionSignal = talon.position
    velocitySignal = talon.velocity
    statorCurrentSignal = talon.statorCurrent
    supplyCurrentSignal = talon.supplyCurrent
    tempSignal = talon.deviceTemp

    motorVoltage = talon.motorVoltage

    talon.configurator.apply(configs)
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        motorVoltage,
        positionSignal,
        velocitySignal,
        tempSignal,
        statorCurrentSignal,
        supplyCurrentSignal)
  }

  override fun updateInputs(inputs: ClimbIO.ClimbInputs) {
    updateSignals()

    inputs.climbPosition = sensor.position

    inputs.temperature = tempSignal.valueAsDouble.celsius
    inputs.supplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.statorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.appliedVoltage = motorVoltage.valueAsDouble.volts
  }

  override fun configPID(
      kP: ProportionalGain<Meter, Volt>,
      kI: IntegralGain<Meter, Volt>,
      kD: DerivativeGain<Meter, Volt>
  ) {
    slot0Configs.kP = kP.inVoltsPerInch
    slot0Configs.kI = kI.inVoltsPerInchSeconds
    slot0Configs.kD = kD.inVoltsPerInchPerSecond

    talon.configurator.apply(slot0Configs)
  }

  override fun configFF(kS: ElectricalPotential, kG: ElectricalPotential) {
    slot0Configs.kS = kS.inVolts
    slot0Configs.kG = kG.inVolts

    talon.configurator.apply(slot0Configs)
  }

  override fun setPosition(position: Length) {
    talon.setControl(
        motionMagicControl.withPosition(sensor.positionToRawUnits(position)).withSlot(0))
  }

  override fun setVoltage(targetVoltage: ElectricalPotential) {
    talon.setControl(voltageControl.withOutput(targetVoltage.inVolts))
  }

  override fun zeroEncoder() {
    talon.setPosition(0.0)
  }
}

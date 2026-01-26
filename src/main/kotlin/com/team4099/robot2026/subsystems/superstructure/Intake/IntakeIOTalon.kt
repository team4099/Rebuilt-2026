package com.team4099.robot2026.subsystems.superstructure.Intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.IntakeConstants
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.AccelerationFeedforward
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianSeconds
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.volts

object IntakeIOTalon : IntakeIO {
  private val intakeTalon: TalonFX = TalonFX(Constants.Intake.INTAKE_PIVOT_MOTOR_ID)
  private val intakeConfiguration: TalonFXConfiguration = TalonFXConfiguration()
  private val intakeSensor =
      ctreAngularMechanismSensor(
          intakeTalon, IntakeConstants.GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION)

  private var temperatureSignal: StatusSignal<Temperature>
  private var voltageSignal: StatusSignal<Voltage>
  private var velocitySignal: StatusSignal<AngularVelocity>
  private var acelSignal: StatusSignal<AngularAcceleration>
  private var statorCurrentSignal: StatusSignal<Current>
  private var supplyCurrentSignal: StatusSignal<Current>
  private var positionSignal: StatusSignal<Angle>

  init {
    intakeTalon.clearStickyFaults()

    intakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    intakeConfiguration.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.STATOR_CURRENT_LIMIT.inAmperes
    intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    intakeConfiguration.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.SUPPLY_CURRENT_LIMIT.inAmperes

    intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    intakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    intakeTalon.configurator.apply(intakeConfiguration)

    temperatureSignal = intakeTalon.deviceTemp
    voltageSignal = intakeTalon.motorVoltage
    velocitySignal = intakeTalon.velocity
    statorCurrentSignal = intakeTalon.statorCurrent
    supplyCurrentSignal = intakeTalon.supplyCurrent
    acelSignal = intakeTalon.acceleration
    positionSignal = intakeTalon.position
  }

  override fun setBrakeMode(brake: Boolean) {
    val brakeModeConfig = MotorOutputConfigs()

    if (brake) {
      brakeModeConfig.NeutralMode = NeutralModeValue.Brake
    } else {
      brakeModeConfig.NeutralMode = NeutralModeValue.Coast
    }

    intakeTalon.configurator.apply(brakeModeConfig)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
    intakeTalon.setControl(VoltageOut(clampedVoltage.inVolts))
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        statorCurrentSignal,
        supplyCurrentSignal,
        temperatureSignal,
        voltageSignal,
        velocitySignal,
        positionSignal,
        acelSignal)
  }

  override fun configPID(
      kP: ProportionalGain<Radian, Volt>,
      kI: IntegralGain<Radian, Volt>,
      kD: DerivativeGain<Radian, Volt>,
  ) {
    intakeConfiguration.Slot0.kP = kP.inVoltsPerRadian
    intakeConfiguration.Slot0.kI = kI.inVoltsPerRadianSeconds
    intakeConfiguration.Slot0.kD = kD.inVoltsPerRadianPerSecond

    intakeTalon.configurator.apply(intakeConfiguration.Slot0)
  }

  override fun configFF(
      kG: ElectricalPotential,
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    intakeConfiguration.Slot0.kG = kG.inVolts
    intakeConfiguration.Slot0.kS = kS.inVolts
    intakeConfiguration.Slot0.kA = kA.inVoltsPerRadiansPerSecondPerSecond
    intakeConfiguration.Slot0.kV = kV.inVoltsPerRadiansPerSecond

    intakeTalon.configurator.apply(intakeConfiguration.Slot0)
  }

  override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
    updateSignals()
    inputs.velocity = intakeSensor.velocity
    inputs.intakeAppliedVoltage = voltageSignal.valueAsDouble.volts
    inputs.intakeStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.intakeSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.intakeTemperature = temperatureSignal.valueAsDouble.celsius
  }

  override fun zeroPivot() {
    intakeTalon.setPosition(IntakeConstants.ZERO_OFFSET.inDegrees)
  }
}

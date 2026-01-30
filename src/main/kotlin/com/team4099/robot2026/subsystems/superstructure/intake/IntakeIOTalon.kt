package com.team4099.robot2026.subsystems.superstructure.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.IntakeConstants
import edu.wpi.first.units.measure.Angle as WPIAngle
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
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.StaticFeedforward
import org.team4099.lib.units.derived.VelocityFeedforward
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerRadian
import org.team4099.lib.units.derived.inVoltsPerRadianPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadianSeconds
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond

object IntakeIOTalon : IntakeIO {
  private val intakeTalon: TalonFX = TalonFX(Constants.Intake.INTAKE_PIVOT_MOTOR_ID)
  private val intakeConfiguration: TalonFXConfiguration = TalonFXConfiguration()
  private val intakeSensor =
      ctreAngularMechanismSensor(
          intakeTalon, IntakeConstants.GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION)
  private val voltageControl = VoltageOut(-1337.volts.inVolts)
  private val motionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)

  private var temperatureSignal: StatusSignal<Temperature>
  private var voltageSignal: StatusSignal<Voltage>
  private var velocitySignal: StatusSignal<AngularVelocity>
  private var acelSignal: StatusSignal<AngularAcceleration>
  private var statorCurrentSignal: StatusSignal<Current>
  private var supplyCurrentSignal: StatusSignal<Current>
  private var positionSignal: StatusSignal<WPIAngle>

  init {
    intakeTalon.clearStickyFaults()

    intakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    intakeConfiguration.CurrentLimits.StatorCurrentLimit =
        IntakeConstants.STATOR_CURRENT_LIMIT.inAmperes
    intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    intakeConfiguration.CurrentLimits.SupplyCurrentLimit =
        IntakeConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    intakeConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine

    intakeConfiguration.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MAX_VELOCITY.inDegreesPerSecond
    intakeConfiguration.MotionMagic.MotionMagicAcceleration = IntakeConstants.MAX_ACCELERATION.inDegreesPerSecondPerSecond

    intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    intakeConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
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
    intakeTalon.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
    intakeTalon.setControl(voltageControl.withOutput(clampedVoltage.inVolts))
  }

  override fun setPosition(position: Angle) {
    intakeTalon.setControl(motionMagicVoltage.withPosition(intakeSensor.positionToRawUnits(position)))
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
    intakeTalon.setPosition(intakeSensor.positionToRawUnits(IntakeConstants.ANGLES.STOW_ANGLE))
  }
}

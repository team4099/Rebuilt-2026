package com.team4099.robot2026.subsystems.superstructure.hopper

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.HopperConstants
import edu.wpi.first.units.measure.AngularAcceleration as WPIAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPIAngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature as WPITemp
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Fraction
import org.team4099.lib.units.base.Second
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
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerDegreesPerSecondPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadians
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecond
import org.team4099.lib.units.derived.inVoltsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerSecond
import org.team4099.lib.units.perSecond

object HopperIOTalon : HopperIO {
  private val hopperTalon: TalonFX = TalonFX(Constants.Hopper.HOPPER_MOTOR_ID)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs: Slot0Configs = configs.Slot0
  private val hopperSensor =
      ctreAngularMechanismSensor(
          hopperTalon, HopperConstants.GEAR_RATIO, HopperConstants.VOLTAGE_COMPENSATION)

  var statorCurrentSignal: StatusSignal<Current>
  var supplyCurrentSignal: StatusSignal<Voltage>
  var tempSignal: StatusSignal<WPITemp>
  var voltageSignal: StatusSignal<Voltage>
  var accelSignal: StatusSignal<WPIAngularAcceleration>
  var velocitySignal: StatusSignal<WPIAngularVelocity>

  val voltageOut = VoltageOut(0.volts.inVolts) // .withEnableFOC(true)
  val velocityControl = MotionMagicVelocityVoltage(0.rotations.perSecond.inRotationsPerSecond)

  init {
    hopperTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = HopperConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = HopperConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive

    hopperTalon.configurator.apply(configs)

    velocitySignal = hopperTalon.velocity
    statorCurrentSignal = hopperTalon.supplyCurrent
    supplyCurrentSignal = hopperTalon.supplyVoltage
    tempSignal = hopperTalon.deviceTemp
    voltageSignal = hopperTalon.motorVoltage
    accelSignal = hopperTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        statorCurrentSignal,
        supplyCurrentSignal,
        tempSignal,
        voltageSignal,
        accelSignal,
        velocitySignal)
  }

  override fun updateInputs(inputs: HopperIO.HopperIOInputs) {
    updateSignals()

    inputs.hopperAppliedVoltage = voltageSignal.valueAsDouble.volts
    inputs.hopperStatorCurrent = statorCurrentSignal.valueAsDouble.amps
    inputs.hopperSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
    inputs.hopperTemp = tempSignal.valueAsDouble.celsius
    inputs.hopperAngularVelocity = hopperSensor.velocity
    inputs.hopperAngularAcceleration =
        (accelSignal.valueAsDouble * HopperConstants.GEAR_RATIO).rotations.perSecond.perSecond
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            lowerBound = -HopperConstants.VOLTAGE_COMPENSATION,
            upperBound = HopperConstants.VOLTAGE_COMPENSATION)
    hopperTalon.setControl(voltageOut.withOutput(clampedVoltage.inVolts))
  }

  override fun setVelocity(velocity: AngularVelocity) {
    hopperTalon.setControl(
        velocityControl
            .withVelocity(hopperSensor.velocityToRawUnits(velocity))
            .withAcceleration(
                hopperSensor.accelerationToRawUnits(HopperConstants.MAX_ACCELERATION)))
  }

  override fun configurePID(
      kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
      kI: IntegralGain<Fraction<Radian, Second>, Volt>,
      kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    slot0Configs.kP = kP.inVoltsPerRadiansPerSecond
    slot0Configs.kI = kI.inVoltsPerRadians
    slot0Configs.kD = kD.inVoltsPerDegreesPerSecondPerSecond
    hopperTalon.configurator.apply(slot0Configs)
  }

  override fun configureFF(
      kS: StaticFeedforward<Volt>,
      kV: VelocityFeedforward<Radian, Volt>,
      kA: AccelerationFeedforward<Radian, Volt>
  ) {
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = kV.inVoltsPerRadiansPerSecond
    slot0Configs.kA = kA.inVoltsPerRadiansPerSecondPerSecond
    hopperTalon.configurator.apply(slot0Configs)
  }

  override fun setBrakeMode(brake: Boolean) {
    hopperTalon.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
  }
}

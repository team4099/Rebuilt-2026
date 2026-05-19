package com.team4099.robot2026.subsystems.superstructure.hopper

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
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
import org.team4099.lib.units.base.Ampere
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
import org.team4099.lib.units.derived.inAmpsPerRadianPerSecond
import org.team4099.lib.units.derived.inAmpsPerRadians
import org.team4099.lib.units.derived.inAmpsPerRadiansPerSecond
import org.team4099.lib.units.derived.inAmpsPerRadiansPerSecondPerSecond
import org.team4099.lib.units.derived.inVolts
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
  var torqueCurrentSignal: StatusSignal<Current>
  var tempSignal: StatusSignal<WPITemp>
  var voltageSignal: StatusSignal<Voltage>
  var accelSignal: StatusSignal<WPIAngularAcceleration>
  var velocitySignal: StatusSignal<WPIAngularVelocity>

  val voltageOut = VoltageOut(0.volts.inVolts) // .withEnableFOC(true)
  val velocityControl =
      MotionMagicVelocityTorqueCurrentFOC(0.rotations.perSecond.inRotationsPerSecond)

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
    torqueCurrentSignal = hopperTalon.torqueCurrent
    tempSignal = hopperTalon.deviceTemp
    voltageSignal = hopperTalon.motorVoltage
    accelSignal = hopperTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        statorCurrentSignal,
        supplyCurrentSignal,
        torqueCurrentSignal,
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
    inputs.hopperTorqueCurrent = torqueCurrentSignal.valueAsDouble.amps
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

  override fun configurePIDCurrent(
      kP: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD: DerivativeGain<Fraction<Radian, Second>, Ampere>
  ) {
    slot0Configs.kP = kP.inAmpsPerRadianPerSecond
    slot0Configs.kI = kI.inAmpsPerRadians
    slot0Configs.kD = kD.inAmpsPerRadiansPerSecondPerSecond
    hopperTalon.configurator.apply(slot0Configs)
  }

  override fun configureFFCurrent(
      kS: StaticFeedforward<Ampere>,
      kV: VelocityFeedforward<Radian, Ampere>,
      kA: AccelerationFeedforward<Radian, Ampere>
  ) {
    slot0Configs.kS = kS.inAmperes
    slot0Configs.kV = kV.inAmpsPerRadiansPerSecond
    slot0Configs.kA = kA.inAmpsPerRadiansPerSecondPerSecond
    hopperTalon.configurator.apply(slot0Configs)
  }

  override fun setBrakeMode(brake: Boolean) {
    hopperTalon.setNeutralMode(if (brake) NeutralModeValue.Brake else NeutralModeValue.Coast)
  }
}

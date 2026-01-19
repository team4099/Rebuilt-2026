package com.team4099.robot2025.subsystems.superstructure.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.SlotConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.ShooterConstants
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
import org.team4099.lib.units.perSecond
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ShooterIOTalon : ShooterIO {
  private val leaderTalon: TalonFX = TalonFX(Constants.Shooter.LEADER_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Shooter.FOLLOWER_MOTOR_ID)
  private val motionMagicControl: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(-1337.0)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val leaderSensor =
    ctreAngularMechanismSensor(
      leaderTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION
    )
  private val followerSensor =
    ctreAngularMechanismSensor(
      followerTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION
    )

  private var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTempSignal: StatusSignal<WPILibTemperature>
  private var leaderDutyCycleSignal: StatusSignal<Double>
  private var leaderAccelSignal: StatusSignal<WPILibAngularAcceleration>

  private var leaderVoltageSignal: StatusSignal<WPILibVoltage>

  private var leaderVelocitySignal: StatusSignal<WPILibAngularVelocity>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTempSignal: StatusSignal<WPILibTemperature>
  private var followerDutyCycleSignal: StatusSignal<Double>

  private var followerVoltageSignal: StatusSignal<WPILibVoltage>
  private var followerAccelSignal: StatusSignal<WPILibAngularAcceleration>

  private var followerVelocitySignal: StatusSignal<WPILibAngularVelocity>

  init {
    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    configs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT.inAmperes
    configs.CurrentLimits.SupplyCurrentLimitEnable = true
    configs.CurrentLimits.StatorCurrentLimitEnable = true
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast

    configs.MotionMagic.MotionMagicAcceleration =
      leaderSensor.accelerationToRawUnits(ShooterConstants.MAX_ACCELERATION)

    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)

    leaderSupplyCurrentSignal = leaderTalon.supplyCurrent
    leaderStatorCurrentSignal = leaderTalon.statorCurrent
    leaderVelocitySignal = leaderTalon.velocity
    leaderDutyCycleSignal = leaderTalon.dutyCycle
    leaderTempSignal = leaderTalon.deviceTemp
    leaderVoltageSignal = leaderTalon.motorVoltage
    leaderAccelSignal = leaderTalon.acceleration

    followerSupplyCurrentSignal = followerTalon.supplyCurrent
    followerStatorCurrentSignal = followerTalon.statorCurrent
    followerVelocitySignal = followerTalon.velocity
    followerDutyCycleSignal = followerTalon.dutyCycle
    followerTempSignal = followerTalon.deviceTemp
    followerVoltageSignal = followerTalon.motorVoltage
    followerAccelSignal = followerTalon.acceleration
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
      leaderSupplyCurrentSignal,
      leaderStatorCurrentSignal,
      leaderVelocitySignal,
      leaderDutyCycleSignal,
      leaderTempSignal,
      leaderVoltageSignal,
      leaderAccelSignal,
      followerStatorCurrentSignal,
      followerSupplyCurrentSignal,
      followerVelocitySignal,
      followerDutyCycleSignal,
      followerTempSignal,
      followerVoltageSignal,
      followerAccelSignal,
    )
  }

  override fun updateInputs(inputs: ShooterIO.ShooterInputs) {
    updateSignals()

    inputs.shooterLeaderVelocity = leaderSensor.velocity
    inputs.shooterFollowerVelocity = followerSensor.velocity
    inputs.shooterLeaderAcceleration =
      (followerAccelSignal.valueAsDouble / ShooterConstants.GEAR_RATIO)
        .rotations
        .perSecond
        .perSecond
    inputs.shooterFollowerAcceleration =
      (followerAccelSignal.valueAsDouble / ShooterConstants.GEAR_RATIO)
        .rotations
        .perSecond
        .perSecond

    inputs.shooterLeaderTemperature = leaderTempSignal.valueAsDouble.celsius
    inputs.shooterLeaderSupplyCurrent = leaderSupplyCurrentSignal.valueAsDouble.amps
    inputs.shooterLeaderStatorCurrent = leaderStatorCurrentSignal.valueAsDouble.amps
    inputs.shooterLeaderVoltage = leaderVoltageSignal.valueAsDouble.volts

    inputs.shooterFollowerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.shooterFollowerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerStatorCurrent = followerStatorCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerVoltage = followerVoltageSignal.valueAsDouble.volts
  }

  override fun configurePID(
    kP: ProportionalGain<Fraction<Radian, Second>, Volt>,
    kI: IntegralGain<Fraction<Radian, Second>, Volt>,
    kD: DerivativeGain<Fraction<Radian, Second>, Volt>
  ) {
    val slot0Configs = SlotConfigs()
    slot0Configs.kP = kP.inVoltsPerRadiansPerSecond
    slot0Configs.kI = kI.inVoltsPerRadians
    slot0Configs.kD = kD.inVoltsPerDegreesPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun configureFF(
    kS: StaticFeedforward<Volt>,
    kV: VelocityFeedforward<Radian, Volt>,
    kA: AccelerationFeedforward<Radian, Volt>
  ) {
    val slot0Configs = SlotConfigs()
    slot0Configs.kS = kS.inVolts
    slot0Configs.kV = kV.inVoltsPerRadiansPerSecond
    slot0Configs.kA = kA.inVoltsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage,
        lowerBound = -ShooterConstants.VOLTAGE_COMPENSATION,
        upperBound = ShooterConstants.VOLTAGE_COMPENSATION
      )
    leaderTalon.setVoltage(clampedVoltage.inVolts)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    leaderTalon.setControl(
      motionMagicControl.withVelocity(leaderSensor.velocityToRawUnits(velocity)),
    )
  }
}

package com.team4099.robot2026.subsystems.superstructure.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.ShooterConstants
import com.team4099.robot2026.util.CustomLogger
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
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
import org.team4099.lib.units.perSecond

object ShooterIOTalon : ShooterIO {
  private val leaderTalon: TalonFX = TalonFX(Constants.Shooter.LEADER_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Shooter.FOLLOWER_MOTOR_ID)
  private val motionMagicControl: MotionMagicVelocityTorqueCurrentFOC =
      MotionMagicVelocityTorqueCurrentFOC(-1337.0)
  private val voltReq = VoltageOut(0.0).withEnableFOC(true)
  private val configs: TalonFXConfiguration = TalonFXConfiguration()
  private val slot0Configs: Slot0Configs = configs.Slot0
  private val slot1Configs: Slot1Configs = configs.Slot1
  private val leaderSensor =
      ctreAngularMechanismSensor(
          leaderTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION)
  private val followerSensor =
      ctreAngularMechanismSensor(
          followerTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION)

  private var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTorqueCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTempSignal: StatusSignal<WPILibTemperature>
  private var leaderVoltageSignal: StatusSignal<WPILibVoltage>
  private var leaderAccelSignal: StatusSignal<WPILibAngularAcceleration>
  private var leaderVelocitySignal: StatusSignal<WPILibAngularVelocity>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTorqueCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var followerTempSignal: StatusSignal<WPILibTemperature>
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
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)

    leaderSupplyCurrentSignal = leaderTalon.supplyCurrent
    leaderTorqueCurrentSignal = leaderTalon.torqueCurrent
    leaderStatorCurrentSignal = leaderTalon.statorCurrent
    leaderVelocitySignal = leaderTalon.velocity
    leaderTempSignal = leaderTalon.deviceTemp
    leaderVoltageSignal = leaderTalon.motorVoltage
    leaderAccelSignal = leaderTalon.acceleration

    followerSupplyCurrentSignal = followerTalon.supplyCurrent
    followerTorqueCurrentSignal = followerTalon.torqueCurrent
    followerStatorCurrentSignal = followerTalon.statorCurrent
    followerVelocitySignal = followerTalon.velocity
    followerTempSignal = followerTalon.deviceTemp
    followerVoltageSignal = followerTalon.motorVoltage
    followerAccelSignal = followerTalon.acceleration

    followerTalon.setControl(
        Follower(Constants.Shooter.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed))
  }

  private fun updateSignals() {
    BaseStatusSignal.refreshAll(
        leaderSupplyCurrentSignal,
        leaderStatorCurrentSignal,
        leaderTorqueCurrentSignal,
        leaderVelocitySignal,
        leaderTempSignal,
        leaderVoltageSignal,
        leaderAccelSignal,
        followerStatorCurrentSignal,
        followerTorqueCurrentSignal,
        followerSupplyCurrentSignal,
        followerVelocitySignal,
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
    inputs.shooterLeaderTorqueCurrent = leaderTorqueCurrentSignal.valueAsDouble.amps
    inputs.shooterLeaderVoltage = leaderVoltageSignal.valueAsDouble.volts

    inputs.shooterFollowerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.shooterFollowerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerStatorCurrent = followerStatorCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerStatorCurrent = followerTorqueCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerVoltage = followerVoltageSignal.valueAsDouble.volts
  }

  override fun configurePIDCurrent(
      kP0: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI0: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD0: DerivativeGain<Fraction<Radian, Second>, Ampere>,
      kP1: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI1: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD1: DerivativeGain<Fraction<Radian, Second>, Ampere>
  ) {
    slot0Configs.kP = kP0.inAmpsPerRadianPerSecond
    slot0Configs.kI = kI0.inAmpsPerRadians
    slot0Configs.kD = kD0.inAmpsPerRadiansPerSecondPerSecond
    slot1Configs.kP = kP1.inAmpsPerRadianPerSecond
    slot1Configs.kI = kI1.inAmpsPerRadians
    slot1Configs.kD = kD1.inAmpsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    leaderTalon.configurator.apply(slot1Configs)
    followerTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot1Configs)
  }

  override fun configureFFCurrent(
      kS0: StaticFeedforward<Ampere>,
      kV0: VelocityFeedforward<Radian, Ampere>,
      kA0: AccelerationFeedforward<Radian, Ampere>,
      kS1: StaticFeedforward<Ampere>,
      kV1: VelocityFeedforward<Radian, Ampere>,
      kA1: AccelerationFeedforward<Radian, Ampere>,
  ) {
    slot0Configs.kS = kS0.inAmperes
    slot0Configs.kV = kV0.inAmpsPerRadiansPerSecond
    slot0Configs.kA = kA0.inAmpsPerRadiansPerSecondPerSecond
    slot1Configs.kS = kS1.inAmperes
    slot1Configs.kV = kV1.inAmpsPerRadiansPerSecond
    slot1Configs.kA = kA1.inAmpsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    leaderTalon.configurator.apply(slot1Configs)
    followerTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot1Configs)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            lowerBound = -ShooterConstants.VOLTAGE_COMPENSATION,
            upperBound = ShooterConstants.VOLTAGE_COMPENSATION)
    leaderTalon.setVoltage(clampedVoltage.inVolts)
  }

  override fun setVelocity(velocity: AngularVelocity) {
    val slotUsed = 0
    //        if (leaderSensor.velocity < velocity - ShooterConstants.SHOOTER_TOLERANCE) 1 else 0
    CustomLogger.recordOutput("Shooter/slotUsed", slotUsed)

    leaderTalon.setControl(
        motionMagicControl
            .withSlot(slotUsed)
            .withVelocity(leaderSensor.velocityToRawUnits(velocity))
            .withAcceleration(
                leaderSensor.accelerationToRawUnits(ShooterConstants.MAX_ACCELERATION)),
    )
  }
}

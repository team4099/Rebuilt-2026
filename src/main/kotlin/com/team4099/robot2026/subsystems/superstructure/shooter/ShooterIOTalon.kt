package com.team4099.robot2026.subsystems.superstructure.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
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
  private val leaderSensor =
      ctreAngularMechanismSensor(
          leaderTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION)
  private val followerSensor =
      ctreAngularMechanismSensor(
          followerTalon, ShooterConstants.GEAR_RATIO, ShooterConstants.VOLTAGE_COMPENSATION)

  private var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  private var leaderTempSignal: StatusSignal<WPILibTemperature>
  private var leaderVoltageSignal: StatusSignal<WPILibVoltage>
  private var leaderAccelSignal: StatusSignal<WPILibAngularAcceleration>
  private var leaderVelocitySignal: StatusSignal<WPILibAngularVelocity>

  private var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
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
    configs.TorqueCurrent.PeakReverseTorqueCurrent =
        ShooterConstants.MAX_REVERSE_TORQUE_CURRENT.inAmperes

    leaderTalon.configurator.apply(configs)
    followerTalon.configurator.apply(configs)

    leaderSupplyCurrentSignal = leaderTalon.supplyCurrent
    leaderStatorCurrentSignal = leaderTalon.statorCurrent
    leaderVelocitySignal = leaderTalon.velocity
    leaderTempSignal = leaderTalon.deviceTemp
    leaderVoltageSignal = leaderTalon.motorVoltage
    leaderAccelSignal = leaderTalon.acceleration

    followerSupplyCurrentSignal = followerTalon.supplyCurrent
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
        leaderVelocitySignal,
        leaderTempSignal,
        leaderVoltageSignal,
        leaderAccelSignal,
        followerStatorCurrentSignal,
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
    inputs.shooterLeaderVoltage = leaderVoltageSignal.valueAsDouble.volts

    inputs.shooterFollowerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.shooterFollowerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerStatorCurrent = followerStatorCurrentSignal.valueAsDouble.amps
    inputs.shooterFollowerVoltage = followerVoltageSignal.valueAsDouble.volts
  }

  override fun configurePIDCurrent(
      kP: ProportionalGain<Fraction<Radian, Second>, Ampere>,
      kI: IntegralGain<Fraction<Radian, Second>, Ampere>,
      kD: DerivativeGain<Fraction<Radian, Second>, Ampere>
  ) {
    slot0Configs.kP = kP.inAmpsPerRadianPerSecond
    slot0Configs.kI = kI.inAmpsPerRadians
    slot0Configs.kD = kD.inAmpsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
  }

  override fun configureFFCurrent(
      kS: StaticFeedforward<Ampere>,
      kV: VelocityFeedforward<Radian, Ampere>,
      kA: AccelerationFeedforward<Radian, Ampere>
  ) {
    slot0Configs.kS = kS.inAmperes
    slot0Configs.kV = kV.inAmpsPerRadiansPerSecond
    slot0Configs.kA = kA.inAmpsPerRadiansPerSecondPerSecond
    leaderTalon.configurator.apply(slot0Configs)
    followerTalon.configurator.apply(slot0Configs)
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
    leaderTalon.setControl(
        motionMagicControl
            .withVelocity(leaderSensor.velocityToRawUnits(velocity))
            .withAcceleration(
                leaderSensor.accelerationToRawUnits(ShooterConstants.MAX_ACCELERATION)),
    )
  }
}

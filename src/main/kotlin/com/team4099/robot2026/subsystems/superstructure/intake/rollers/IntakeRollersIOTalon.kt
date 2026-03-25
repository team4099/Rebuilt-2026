package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.team4099.lib.math.clamp
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.RollersConstants
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.Temperature
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

  private val leaderTalon: TalonFX = TalonFX(Constants.Intake.LEADER_ROLLERS_MOTOR_ID)
  private val followerTalon: TalonFX = TalonFX(Constants.Intake.LEADER_ROLLERS_MOTOR_ID)

  private val leaderConfig: TalonFXConfiguration = TalonFXConfiguration()
  private val followerConfig: TalonFXConfiguration = TalonFXConfiguration()

  val voltageControl = VoltageOut(0.0.volts.inVolts).withEnableFOC(true)

  private var leaderSensor =
      ctreAngularMechanismSensor(
          leaderTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION)
  private var followerSensor =
      ctreAngularMechanismSensor(
          followerTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION)

  var leaderStatorCurrentSignal: StatusSignal<Current>
  var leaderSupplyCurrentSignal: StatusSignal<Current>
  var leaderTempSignal: StatusSignal<Temperature>
  var leaderVoltageSignal: StatusSignal<Voltage>
  var leaderAccelSignal: StatusSignal<AngularAcceleration>
  var leaderVelocitySignal: StatusSignal<AngularVelocity>

  var followerStatorCurrentSignal: StatusSignal<Current>
  var followerSupplyCurrentSignal: StatusSignal<Current>
  var followerTempSignal: StatusSignal<Temperature>
  var followerVoltageSignal: StatusSignal<Voltage>
  var followerAccelSignal: StatusSignal<AngularAcceleration>
  var followerVelocitySignal: StatusSignal<AngularVelocity>

  init {
    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    leaderConfig.CurrentLimits.SupplyCurrentLimit = RollersConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    leaderConfig.CurrentLimits.StatorCurrentLimit = RollersConstants.STATOR_CURRENT_LIMIT.inAmperes
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

    followerConfig.CurrentLimits.SupplyCurrentLimit =
        RollersConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    followerConfig.CurrentLimits.StatorCurrentLimit =
        RollersConstants.STATOR_CURRENT_LIMIT.inAmperes
    followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive

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

    leaderTalon.configurator.apply(leaderConfig)
    followerTalon.configurator.apply(followerConfig)

    followerTalon.setControl(
        (Follower(Constants.Intake.LEADER_ROLLERS_MOTOR_ID, MotorAlignmentValue.Aligned)))
  }

  override fun updateInputs(inputs: IntakeRollersIO.RollerInputs) {
    refreshStatusSignals()

    inputs.leaderVelocity = leaderSensor.velocity
    inputs.leaderAppliedVoltage = leaderVoltageSignal.valueAsDouble.volts
    inputs.leaderStatorCurrent = leaderSupplyCurrentSignal.valueAsDouble.amps
    inputs.leaderSupplyCurrent = leaderSupplyCurrentSignal.valueAsDouble.amps
    inputs.leaderTemperature = leaderTempSignal.valueAsDouble.celsius
    inputs.leaderAcceleration =
        leaderAccelSignal.valueAsDouble.rotations.perSecond.perSecond * RollersConstants.GEAR_RATIO
    inputs.followerVelocity = followerSensor.velocity
    inputs.followerAppliedVoltage = followerVoltageSignal.valueAsDouble.volts
    inputs.followerStatorCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.followerSupplyCurrent = followerSupplyCurrentSignal.valueAsDouble.amps
    inputs.followerTemperature = followerTempSignal.valueAsDouble.celsius
    inputs.followerAcceleration =
        followerAccelSignal.valueAsDouble.rotations.perSecond.perSecond *
            RollersConstants.GEAR_RATIO
  }

  fun refreshStatusSignals() {
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
        followerAccelSignal)
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
        clamp(
            voltage,
            (-RollersConstants.VOLTAGE_COMPENSATION),
            RollersConstants.VOLTAGE_COMPENSATION)
    leaderTalon.setVoltage(clampedVoltage.inVolts)
  }
}

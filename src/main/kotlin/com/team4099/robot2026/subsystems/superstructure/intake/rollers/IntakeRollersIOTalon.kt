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
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import edu.wpi.first.units.measure.AngularAcceleration as WPILibAngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity as WPILibAngularVelocity
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
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

  private val rollerConfig: TalonFXConfiguration = TalonFXConfiguration()

  val voltageControl = VoltageOut(0.0.volts.inVolts).withEnableFOC(true)

  private var leaderSensor =
    ctreAngularMechanismSensor(
      leaderTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION
    )
  private var followerSensor =
    ctreAngularMechanismSensor(
      followerTalon, RollersConstants.GEAR_RATIO, RollersConstants.VOLTAGE_COMPENSATION
    )

  var leaderStatorCurrentSignal: StatusSignal<WPILibCurrent>
  var leaderSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  var leaderTempSignal: StatusSignal<WPILibTemperature>
  var leaderVoltageSignal: StatusSignal<WPILibVoltage>
  var leaderAccelSignal: StatusSignal<WPILibAngularAcceleration>
  var leaderVelocitySignal: StatusSignal<WPILibAngularVelocity>

  var followerStatorCurrentSignal: StatusSignal<WPILibCurrent>
  var followerSupplyCurrentSignal: StatusSignal<WPILibCurrent>
  var followerTempSignal: StatusSignal<WPILibTemperature>
  var followerVoltageSignal: StatusSignal<WPILibVoltage>
  var followerAccelSignal: StatusSignal<WPILibAngularAcceleration>
  var followerVelocitySignal: StatusSignal<WPILibAngularVelocity>

  init {
    leaderTalon.clearStickyFaults()
    followerTalon.clearStickyFaults()

    rollerConfig.CurrentLimits.SupplyCurrentLimit = RollersConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    rollerConfig.CurrentLimits.StatorCurrentLimit = RollersConstants.STATOR_CURRENT_LIMIT.inAmperes
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive



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

    leaderTalon.configurator.apply(rollerConfig)
    followerTalon.configurator.apply(rollerConfig)

    followerTalon.setControl(
      (Follower(Constants.Intake.LEADER_ROLLERS_MOTOR_ID, MotorAlignmentValue.Opposed))
    )
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
      followerAccelSignal
    )
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    val clampedVoltage =
      clamp(
        voltage,
        (-RollersConstants.VOLTAGE_COMPENSATION),
        RollersConstants.VOLTAGE_COMPENSATION
      )
    leaderTalon.setVoltage(clampedVoltage.inVolts)
  }
}


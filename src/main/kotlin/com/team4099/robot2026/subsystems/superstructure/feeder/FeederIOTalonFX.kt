package com.team4099.robot2025.subsystems.feeders

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.subsystems.Feeders.FeederIO
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.FeederConstants
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts

object FeederIOTalonFX : FeederIO {

  private val feederTalon: TalonFX = TalonFX(Constants.feeder.FEEDER_MOTOR_ID)
  private val feederConfiguration: TalonFXConfiguration = TalonFXConfiguration()

  private val feederSensor =
      ctreAngularMechanismSensor(
          feederTalon, FeederConstants.GEAR_RATIO, FeederConstants.VOLTAGE_COMPENSATION)

  var feederAppliedVoltageStatusSignal: StatusSignal<WPILibVoltage>
  var feederStatorCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var feederSupplyCurrentStatusSignal: StatusSignal<WPILibCurrent>
  var feederTempStatusSignal: StatusSignal<WPILibTemperature>

  val voltageControl: VoltageOut = VoltageOut(0.volts.inVolts)

  init {

    // configurations
    feederConfiguration.CurrentLimits.StatorCurrentLimit =
        FeederConstants.STATOR_CURRENT_LIMIT.inAmperes
    feederConfiguration.CurrentLimits.SupplyCurrentLimit =
        FeederConstants.SUPPLY_CURRENT_LIMIT.inAmperes
    feederConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
    feederConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
    feederConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
    feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake

    feederTalon.configurator.apply(feederConfiguration)

    // sensor data
    feederAppliedVoltageStatusSignal = feederTalon.motorVoltage
    feederStatorCurrentStatusSignal = feederTalon.statorCurrent
    feederSupplyCurrentStatusSignal = feederTalon.supplyCurrent
    feederTempStatusSignal = feederTalon.deviceTemp
  }

  fun refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        feederAppliedVoltageStatusSignal,
        feederStatorCurrentStatusSignal,
        feederSupplyCurrentStatusSignal,
        feederTempStatusSignal)
  }

  override fun updateInputs(inputs: FeederIO.FeederIOInputs) {
    refreshStatusSignals()
    inputs.feederVelocity = feederSensor.velocity
    inputs.feederAppliedVoltage = feederAppliedVoltageStatusSignal.valueAsDouble.volts
    inputs.feederStatorCurrent = feederStatorCurrentStatusSignal.valueAsDouble.amps
    inputs.feederSupplyCurrent = feederSupplyCurrentStatusSignal.valueAsDouble.amps
    inputs.feederTemp = feederTempStatusSignal.valueAsDouble.celsius
    // inputs.beamBroken = beamBreakStatusSignal.value
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    feederTalon.setControl(voltageControl.withOutput(voltage.inVolts))
  }

  override fun setBrakeMode(brake: Boolean) {
    if (brake) {
      feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
    } else {
      feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast
    }

    feederTalon.configurator.apply(feederConfiguration)
  }
}

package com.team4099.robot2025.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2025.config.constants.Constants
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.IntakeConstants
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import com.ctre.phoenix6.controls.VoltageOut

/**
 * Hardware implementation for intake rollers with any TalonFX compatible motor. This class takes the values for things like voltage, velocity, etc. from the motor and sends them to the IntakeInputs class through the updateInputs() function, allowing IntakeInputs to always have updated values.
 *
 * @author Aryan Singh
 * @author Ayan Shukla
 */
object IntakeIOTalon : IntakeIO {
    private val intakeTalon: TalonFX = TalonFX(Constants.Intake.INTAKE_ROLLERS_MOTOR_ID)
    private val intakeConfiguration: TalonFXConfiguration = TalonFXConfiguration()
    private val intakeSensor = ctreAngularMechanismSensor(intakeTalon, IntakeConstants.GEAR_RATIO, IntakeConstants.VOLTAGE_COMPENSATION)

    private var temperatureSignal: StatusSignal<Temperature>
    private var voltageSignal: StatusSignal<Voltage>
    private var velocitySignal : StatusSignal<AngularVelocity>
    private var statorCurrentSignal: StatusSignal<Current>
    private var supplyCurrentSignal: StatusSignal<Current>

    // "init" block is run at compile time: all the code in this
    init {
        intakeTalon.clearStickyFaults()

        intakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true
        intakeConfiguration.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT.inAmperes
        intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true
        intakeConfiguration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        intakeConfiguration.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.THRESHOLD_CURRENT_LIMIT.inAmperes

        intakeConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true
        intakeConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true

        intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake
        intakeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive
        intakeTalon.configurator.apply(intakeConfiguration)

        temperatureSignal = intakeTalon.deviceTemp
        voltageSignal = intakeTalon.motorVoltage
        velocitySignal = intakeTalon.velocity
        statorCurrentSignal = intakeTalon.statorCurrent
        supplyCurrentSignal = intakeTalon.supplyCurrent
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
        val clampedVoltage = clamp(voltage, -IntakeConstants.VOLTAGE_COMPENSATION, IntakeConstants.VOLTAGE_COMPENSATION)
        intakeTalon.setControl(VoltageOut(clampedVoltage.inVolts))
    }

    private fun updateSignals(){
        BaseStatusSignal.refreshAll(
                statorCurrentSignal,
                supplyCurrentSignal,
                temperatureSignal,
                voltageSignal,
                velocitySignal
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        updateSignals()

        intakeTalon.rotorPosition.refresh()
        intakeTalon.position.refresh()

        inputs.intakeVelocity = intakeSensor.velocity
        inputs.intakeAppliedVoltage = voltageSignal.valueAsDouble.volts
        inputs.intakeStatorCurrent = statorCurrentSignal.valueAsDouble.amps
        inputs.intakeSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
        inputs.intakeTemperature = temperatureSignal.valueAsDouble.celsius
    }
}
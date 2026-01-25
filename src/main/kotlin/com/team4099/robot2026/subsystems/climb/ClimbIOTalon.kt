package com.team4099.robot2026.subsystems.climb

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2026.config.constants.ClimbConstants
import com.team4099.robot2026.config.constants.ClimbConstants.MAX_ACCELERATION
import com.team4099.robot2026.config.constants.ClimbConstants.MAX_VELOCITY
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.Logger
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Meter
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inInches
import org.team4099.lib.units.derived.DerivativeGain
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.IntegralGain
import org.team4099.lib.units.derived.ProportionalGain
import org.team4099.lib.units.derived.Volt
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.inVoltsPerInch
import org.team4099.lib.units.derived.inVoltsPerInchPerSecond
import org.team4099.lib.units.derived.inVoltsPerInchSeconds
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.ctreLinearMechanismSensor
import org.team4099.lib.units.inInchesPerSecond
import org.team4099.lib.units.inInchesPerSecondPerSecond
import kotlin.time.times
import edu.wpi.first.units.measure.Current as WPILibCurrent
import edu.wpi.first.units.measure.Temperature as WPILibTemperature
import edu.wpi.first.units.measure.Voltage as WPILibVoltage

object ClimbIOTalon : ClimbIO {
    private val talon: TalonFX = TalonFX(Constants.Climb.MOTOR_ID)

    private val configs: TalonFXConfiguration = TalonFXConfiguration()
    private val slot0Configs = configs.Slot0

    private val voltageControl: VoltageOut = VoltageOut(-1337.volts.inVolts)
    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.0)

    private val sensor =
        ctreLinearMechanismSensor(
            talon,
            ClimbConstants.GEAR_RATIO,
            ClimbConstants.DRUM_DIAMETER,
            ClimbConstants.VOLTAGE_COMPENSATION
        )

    private var statorCurrentSignal: StatusSignal<WPILibCurrent>
    private var supplyCurrentSignal: StatusSignal<WPILibCurrent>
    private var tempSignal: StatusSignal<WPILibTemperature>
    private var dutyCycle: StatusSignal<Double>
    private var positionSignal: StatusSignal<edu.wpi.first.units.measure.Angle>
    private var velocitySignal: StatusSignal<AngularVelocity>

    private var motorVoltage: StatusSignal<WPILibVoltage>
    private var motorTorque: StatusSignal<WPILibCurrent>

    private var motionMagicTargetVelocity: StatusSignal<Double>
    private var motionMagicTargetPosition: StatusSignal<Double>

    init {
        talon.clearStickyFaults()

        configs.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        configs.CurrentLimits.SupplyCurrentLowerLimit = ClimbConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        configs.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_CURRENT_LIMIT.inAmperes
        configs.CurrentLimits.SupplyCurrentLowerTime = ClimbConstants.SUPPLY_CURRENT_LIMIT.inAmperes
        configs.CurrentLimits.StatorCurrentLimitEnable = true
        configs.CurrentLimits.SupplyCurrentLimitEnable = true

        configs.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY.inInchesPerSecond
        configs.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION.inInchesPerSecondPerSecond

        positionSignal = talon.position
        velocitySignal = talon.velocity
        statorCurrentSignal = talon.statorCurrent
        supplyCurrentSignal = talon.supplyCurrent
        tempSignal = talon.deviceTemp
        dutyCycle = talon.dutyCycle

        motorVoltage = talon.motorVoltage
        motorTorque = talon.torqueCurrent

        motionMagicTargetPosition = talon.closedLoopReference
        motionMagicTargetVelocity = talon.closedLoopReferenceSlope

        motionMagicTargetPosition.setUpdateFrequency(250.0)
        motionMagicTargetVelocity.setUpdateFrequency(250.0)

        talon.configurator.apply(configs)
    }

    private fun updateSignals() {
        BaseStatusSignal.refreshAll(
            motorTorque,
            motorVoltage,
            positionSignal,
            velocitySignal,
            tempSignal,
            dutyCycle,
            statorCurrentSignal,
            supplyCurrentSignal,
            motionMagicTargetPosition,
            motionMagicTargetVelocity
        )
    }

    override fun updateInputs(inputs: ClimbIO.ClimbInputs) {
        updateSignals()

        inputs.climbPosition = sensor.position

        inputs.temperature = tempSignal.valueAsDouble.celsius
        inputs.supplyCurrent = supplyCurrentSignal.valueAsDouble.amps
        inputs.statorCurrent = statorCurrentSignal.valueAsDouble.amps
        inputs.appliedVoltage = (dutyCycle.valueAsDouble * 12).volts

        Logger.recordOutput(
            "Climb/motionMagicPosition",
            motionMagicTargetPosition.value *
                    ClimbConstants.GEAR_RATIO *
                    (Math.PI * ClimbConstants.DRUM_DIAMETER.inInches)
        )
        Logger.recordOutput(
            "Climb/motionMagicVelocity",
            motionMagicTargetVelocity.value *
                    ClimbConstants.GEAR_RATIO *
                    (Math.PI * ClimbConstants.DRUM_DIAMETER.inInches)
        )
    }

    override fun configPID(
        kP: ProportionalGain<Meter, Volt>,
        kI: IntegralGain<Meter, Volt>,
        kD: DerivativeGain<Meter, Volt>
    ) {
        slot0Configs.kP = kP.inVoltsPerInch
        slot0Configs.kI = kI.inVoltsPerInchSeconds
        slot0Configs.kD = kD.inVoltsPerInchPerSecond

        talon.configurator.apply(slot0Configs)
    }

    override fun setPosition(position: Length) {
        talon.setControl(
            motionMagicControl.withPosition(sensor.positionToRawUnits(position)).withSlot(0)
        )
    }

    override fun setVoltage(targetVoltage: ElectricalPotential) {
        talon.setControl(voltageControl.withOutput(targetVoltage.inVolts))
    }

    override fun zeroEncoder() {
        talon.setPosition(0.0)
    }
}

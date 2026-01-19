package com.team4099.robot2025.subsystems.hopper

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2026.subsystems.hopper.HopperIO
import edu.wpi.first.units.measure.Voltage
import org.team4099.lib.units.AngularAcceleration
import org.team4099.lib.units.AngularVelocity
import org.team4099.lib.units.Value
import org.team4099.lib.units.base.Ampere
import org.team4099.lib.units.base.Temperature
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.ctreAngularMechanismSensor
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond

object HopperIOTalon: HopperIO {

    private val hopperTalon: TalonFX = TalonFX(Constants.Hopper.HOPPER_MOTOR_ID)
    private val motionMagicControl: MotionMagicVoltage = MotionMagicVoltage(-1337.degrees.inDegrees)
    private val configs: TalonFXConfiguration = TalonFXConfiguration()
    private val subsystemSensor = ctreAngularMechanismSensor(hopperTalon, Constants.Hopper.GEAR_RATIO,Constants.Hopper.VOLTAGE_COMPENSATION)

    private fun ctreAngularMechanismSensor(
        controller: TalonFX,
        ratio: Double,
        compensationVoltage: Value<Ampere>
    ) {
        TODO("Not yet implemented")
    }

    override fun setBrakeMode(brake: Boolean) {
        TODO("Not yet implemented")
    }
        var statorCurrentSignal: StatusSignal<Ampere>
        var supplyCurrentSignal: StatusSignal<Voltage>
        var tempSignal: StatusSignal<Temperature>
        var dutyCycleSignal: StatusSignal<Double>
        var motorVoltageSignal: StatusSignal<Voltage>
        var motorAccelSignal: StatusSignal<AngularAcceleration>
        var rotorVelocitySignal: StatusSignal<AngularVelocity>
    init {
        hopperTalon.clearStickyFaults()

        //setup these constants for every subsystem (see below for more info)
        configs.CurrentLimits.SupplyCurrentLimit = -1337.0
        configs.CurrentLimits.StatorCurrentLimit = -1337.0
        configs.CurrentLimits.SupplyCurrentLimitEnable = true
        configs.CurrentLimits.StatorCurrentLimitEnable = true
        configs.MotorOutput.NeutralMode = null

        configs.MotionMagic.MotionMagicCruiseVelocity = Constants.Hopper.MAX_VELOCITY//     constants file when
        configs.MotionMagic.MotionMagicAcceleration = Constants.Hopper.MAX_ACCELERATION //  (code reusing mmmm)

        hopperTalon.configurator.apply(configs)

        rotorVelocitySignal = hopperTalon.velocity
        statorCurrentSignal = hopperTalon.supplyCurrent
        supplyCurrentSignal = hopperTalon.supplyVoltage
        tempSignal = hopperTalon.deviceTemp
        dutyCycleSignal = hopperTalon.dutyCycle
        motorVoltageSignal = hopperTalon.motorVoltage
        motorAccelSignal = hopperTalon.acceleration

    }
    private fun updateSignals() {
        BaseStatusSignal.refreshAll(
            statorCurrentSignal,
            supplyCurrentSignal,
            tempSignal,
            dutyCycleSignal,
            motorVoltageSignal,
            motorAccelSignal,
            rotorVelocitySignal
        )
    }

    override fun updateInputs(inputs: HopperIO.HopperIOInputs) {
        updateSignals()
        inputs.hopperAppliedVoltage = motorVoltageSignal.valueAsDouble.volts
        inputs.hopperStatorCurrent = statorCurrentSignal.valueAsDouble.amps
        inputs.hopperSupplyCurrent = supplyCurrentSignal.valueAsDouble.amps
        inputs.hopperTemp = tempSignal.valueAsDouble.celsius
        inputs.hopperAcceleration=
            (motorAccelSignal.valueAsDouble / Constants.Hopper.ENCODER_TO_MECHANISM_GEAR_RATIO)
                .rotations
                .perSecond
                .perSecond
    }

    }
}
package com.team4099.robot2025.subsystems.intake

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inRotationsPerMinute
import org.team4099.lib.units.perMinute

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        var intakeVelocity = 0.rotations.perMinute
        var intakeAppliedVoltage = 0.volts
        var intakeTemperature = 0.0.celsius
        var intakeStatorCurrent = 0.amps
        var intakeSupplyCurrent = 0.amps

        override fun toLog(table: LogTable?) {
            table?.put("intakeTemperatureCelsius", intakeTemperature.inCelsius)
            table?.put("intakeAppliedVolts", intakeAppliedVoltage.inVolts)
            table?.put("intakeVelocityRPM", intakeVelocity.inRotationsPerMinute)
            table?.put("intakeStatorCurrentAmps", intakeStatorCurrent.inAmperes)
            table?.put("intakeSupplyCurrentAmps", intakeSupplyCurrent.inAmperes)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("intakeTemperatureCelsius", intakeTemperature.inCelsius)?.let {
                intakeTemperature = it.celsius
            }
            table?.get("intakeAppliedVolts", intakeAppliedVoltage.inVolts)?.let{
                intakeAppliedVoltage = it.volts
            }
            table?.get("intakeVelocityRPM", intakeVelocity.inRotationsPerMinute)?.let {
                intakeVelocity = it.rotations.perMinute
            }
            table?.get("intakeStatorCurrentAmps", intakeStatorCurrent.inAmperes)?.let{
                intakeStatorCurrent = it.amps
            }
            table?.get("intakeSupplyCurrentAmps", intakeSupplyCurrent.inAmperes)?. let{
                intakeSupplyCurrent = it.amps
            }
        }
    }

    fun updateInputs(inputs: IntakeInputs) {}

    fun setVoltage(voltage: ElectricalPotential){}

    fun setBrakeMode(brake: Boolean) {}
}
package com.team4099.robot2026.subsystems.hopper

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.Current
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.perSecond

interface HopperIO{
    class HopperIOInputs : LoggableInputs {
        //Hopper Inputs
        var hopperVelocity = 0.0.degrees.perSecond
        var hopperAcceleration = 0.0.degrees.perSecond.perSecond
        var hopperAppliedVoltage = 0.0.volts
        var hopperStatorCurrent = 0.0.amps
        var hopperSupplyCurrent = 0.0.amps
        var hopperTemp = 0.0.celsius
        var isSimulating = falase
        override fun toLog(table: LogTable) {
            table.put("hopperVelocityPerSecond", hopperVelocity.inDegreesPerSecond)
        }
        override fun getLog(table: LogTable){
            table.get("hopperVelocityPerSecond", hopperVelocity.inDegreesPerSecond){
                hopperVelocity = it.hopperVelocity.inDegreesPerSecond
            }
        }
        override fun toLog(table: LogTable){
            table.put("hopperAcceleration", hopperAcceleration.inDegrees)
            table.put("hopperAccelerationPerSecond", hopperAcceleration.inDegreesPerSecond.inDegreesPerSecond)
        }

        override fun getLog(table: LogTable){
            table.get("hopperAcceleration", hopperAcceleration.inDegreesPerSecond.inDegreesPerSecond).let{
                hopperAcceleration = it.inDegreesPerSecond.inDegreesPerSecond
            }
        }
        override fun toLog(table: LogTable){
            table.put("hopperAppliedVoltage", hopperAppliedVoltage.inVolts)
            }
        override fun getLog(table: LogTable){
            table.get("hopperAppliedVoltage", hopperAppliedVoltage.inVolts){
                hopperAppliedVoltage = it.inVolts
            }
        }
        override  fun toLog(table: LogTable){
            table.put("hopperStatorCurrent",){

            }
        }
        fun updateInputs(inputs: hopperInputs) {

        }
        fun setVoltage(voltage: hopperSetVoltage) {

        }

        fun setPosition(position: hopperSetPosition) {

        }
        fun setBrakeMode(brake: Boolean) {

        }
        fun configFF(
            kG: ElectricalPotential,
            kS: StaticFeedforward<Volt>,
            kV: VelocityFeedforward<MeasurementUnit, Volt>,
            kA: AccelerationFeedforward<MeasurementUnit, Volt>
        ) {

        }
    }
}
package com.team4099.robot2025.subsystems.shooter

import com.team4099.robot2025.subsystems.superstructure.Request
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inAmperes
import org.team4099.lib.units.base.inCelsius
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.perSecond

interface ShooterIO{
    class ShooterInputs : LoggagleInputs {
        var flywheelSpeed = 0.0.rotations.PerMinute
        //leader variables
        var shooterLeaderVoltage = 0.0.volts
        var shooterLeaderSupplyCurrent = 0.0.amps
        var shooterLeaderStatorCurrent = 0.0.amps
        var shooterLeaderTemperature = 0.0.celsius
        var shooterLeaderVelocity = 0.0.degrees.perSecond
        //follower variables
        var shooterFollowerVoltage = 0.0.volts
        var shooterFollowerSupplyCurrent = 0.0.amps
        var shooterFollowerStatorCurrent = 0.0.amps
        var shooterFollowerTemperature = 0.0.celsius
        var shooterFollowerVelocity = 0.0.degrees.perSecond

        override fun toLog(table: LogTable?){
            table?.put("FlywheelSpeed", flywheelSpeed.inRotations.perMinute)
            //leader
            table?.put("ShooterLeaderVoltage", shooterLeaderVoltage.inVolts)
            table?.put("ShooterLeaderSupplyCurrent", shooterLeaderSupplyCurrent.inAmperes)
            table?.put("ShooterLeaderStatorCurrent", shooterLeaderStatorCurrent.inAmperes)
            table?.put("ShooterLeaderTemperature", shooterLeaderTemperature.inCelsius)
            table?.put("ShooterLeaderVelocity", shooterLeaderVelocity.inVolts)
            //follower
            table?.put("ShooterFollowerVoltage", shooterFollowerVoltage.inVolts)
            table?.put("ShooterFollowerSupplyCurrent", shooterFollowerSupplyCurrent.inAmperes)
            table?.put("ShooterFollowerStatorCurrent", shooterFollowerStatorCurrent.inAmperes)
            table?.put("ShooterFollowerTemperature", shooterFollowerTemperature.inCelsius)
            table?.put("ShooterFollowerVelocity", shooterFollowerVelocity.inVolts)
        }

        override fun fromLog(table: LogTable) {

            table.get("FlywheelSpeed" , flywheelSpeed.inRotations.perMinute).let {
                flywheelSpeed = it.rotations.PerMinute
            }
            //leader
            table.get("ShooterLeaderVoltage" , shooterLeaderVoltage.inVolts).let {
                shooterLeaderVoltage = it.volts
            }
            table.get("ShooterLeaderSupplyCurrent" , shooterLeaderSupplyCurrent.inAmperes).let {
                shooterLeaderSupplyCurrent = it.amps
            }
            table.get("ShooterLeaderStatorCurrent" , shooterLeaderStatorCurrent.inAmperes).let {
                shooterLeaderStatorCurrent = it.amps
            }
            table.get("ShooterLeaderTemperatureCurrent" , shooterLeaderTemperature.inCelsius).let {
                shooterLeaderTemperature = it.Celsius
            }
            table.get("ShooterLeaderVelcoity" , shooterLeaderVelocity.inVolts).let {
                shooterLeaderVelocity = it.volts

            }
            //follower
            table.get("ShooterFollowerVoltage" , shooterFollowerVoltage.inVolts).let {
                shooterFollowerVoltage = it.volts
            }
            table.get("ShooterFollowerSupplyCurrent" , shooterFollowerSupplyCurrent.inAmperes).let {
                shooterFollowerSupplyCurrent = it.amps
            }
            table.get("ShooterFollowerStatorCurrent" , shooterFollowerStatorCurrent.inAmperes).let {
                shooterFollowerStatorCurrent = it.amps
            }
            table.get("ShooterFollowerTemperatureCurrent" , shooterFollowerTemperature.inCelsius).let {
                shooterFollowerTemperature = it.Celsius
            }
            table.get("ShooterFollowerVelcoity" , shooterFollowerVelocity.inVolts).let {
                shooterFollowerVelocity = it.volts
        }

    }
        fun updateInputs(inputs: ShooterInputs) {}
        fun setVoltage(inputs: ElectricalPotential) {}
        fun configurePID(
            kP: ProportionalGain<Meter, Volt>,
            kI: IntegralGain<Meter, Volt>,
            kD: DerivativeGain<Meter, Volt>) {}
        fun configureFF(
            kGFirstStage: ElectricalPotential,
            kGSecondStage: ElectricalPotential,
            kS: StaticFeedforward<Volt>,
            kV: VelocityFeedforward<Meter, Volt>,
            kA: AccelerationFeedforward<Meter, Volt>
        ) {}
        fun setVelocity(inputs: Velocity)
        fun setBreakMode(brake: Boolean) {}
}
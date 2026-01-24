package com.team4099.robot2026.subsystems.superstructure.Intake.Rollers

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.IntakeConstants
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2025.subsystems.superstructure.Intake.Rollers.RollersIO
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.inDegreesPerSecondPerSecond
import org.team4099.lib.units.inRotationsPerMinutePerMinute
import org.team4099.lib.units.perMinute
import org.team4099.lib.units.perSecond

object RollersIOSim : RollersIO {
    private val rollerSim: FlywheelSim = FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(1),
            RollersConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
            1 / RollersConstants.GEAR_RATIO
        ),
        DCMotor.getKrakenX60(1),
        1 / RollersConstants.GEAR_RATIO
    )

    private var appliedVoltage = 0.volts

    override fun updateInputs(inputs: RollersIO.RollerInputs) {
        rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
        inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
        inputs.rollerAcceleration = rollerSim.angularAccelerationRadPerSecSq.radians.perSecond.perSecond.inDegreesPerSecondPerSecond.degrees.perSecond.perSecond
        inputs.rollerAppliedVoltage = appliedVoltage
        inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
        inputs.rollerSupplyCurrent = 0.amps
        inputs.rollerTemperature = 25.celsius
    }

    /**
     * Sets the intake motor voltage and ensures the voltage is within the bounds of the battery voltage compensation
     *
     * @param voltage the voltage to set the roller motor to
     */
    override fun setVoltage(voltage: ElectricalPotential) {
        val clampedVoltage = clamp(voltage, -RollersConstants.VOLTAGE_COMPENSATION, RollersConstants.VOLTAGE_COMPENSATION)
        rollerSim.inputVoltage = clampedVoltage.inVolts
        appliedVoltage = clampedVoltage
    }
}

package com.team4099.robot2026.subsystems.superstructure.Intake.Rollers

import com.team4099.lib.math.clamp
import com.team4099.robot2025.config.constants.RollersConstants
import com.team4099.robot2026.config.constants.Constants
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.rotations
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perMinute

object RollersIOSim : RollersIO {

  private val rollerSim: FlywheelSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              RollersConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1 / RollersConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1),
          1 / RollersConstants.GEAR_RATIO)

  private var appliedVoltage = 0.0.volts

  override fun updateInputs(inputs: RollersIO.RollerInputs) {

    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerAppliedVoltage = appliedVoltage
    inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerSupplyCurrent = 0.0.amps
    inputs.rollerTemp = 25.0.celsius
    inputs.isSimulating = true
  }

  override fun setVoltage(voltage: ElectricalPotential) {

    val clampedVoltage =
        clamp(
            voltage, -RollersConstants.VOLTAGE_COMPENSATION, RollersConstants.VOLTAGE_COMPENSATION)
    rollerSim.setInputVoltage(clampedVoltage.inVolts)
  }
}

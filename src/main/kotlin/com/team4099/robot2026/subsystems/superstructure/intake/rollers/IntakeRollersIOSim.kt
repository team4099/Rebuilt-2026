package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.RollersConstants
import com.team4099.robot2026.subsystems.superstructure.Intake.Rollers.IntakeRollersIO
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
import org.team4099.lib.units.perSecond

object IntakeRollersIOSim : IntakeRollersIO {
  private val rollerSim: FlywheelSim =
      FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1),
              RollersConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1 / RollersConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1),
          1 / RollersConstants.GEAR_RATIO)

  private var appliedVoltage = 0.volts

  override fun updateInputs(inputs: IntakeRollersIO.RollerInputs) {
    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.rollerVelocity = rollerSim.angularVelocityRPM.rotations.perMinute
    inputs.rollerAcceleration = rollerSim.angularVelocityRPM.rotations.perSecond.perSecond
    inputs.rollerAppliedVoltage = appliedVoltage
    inputs.rollerStatorCurrent = rollerSim.currentDrawAmps.amps
    inputs.rollerSupplyCurrent = 0.amps
    inputs.rollerTemperature = 25.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    rollerSim.setInputVoltage(voltage.inVolts)
    appliedVoltage = voltage
  }
}

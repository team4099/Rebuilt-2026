package com.team4099.robot2026.subsystems.superstructure.intake.rollers

import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.config.constants.RollersConstants
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.inKilogramsMeterSquared
import org.team4099.lib.units.derived.inVolts
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.volts
import org.team4099.lib.units.perSecond
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.FlywheelSim

object IntakeRollersIOSim : IntakeRollersIO {
  private val rollerSim: FlywheelSim =
      FlywheelSim(
          Models.flywheelFromPhysicalConstants(
              DCMotor.getKrakenX60(1),
              RollersConstants.MOMENT_OF_INERTIA.inKilogramsMeterSquared,
              1 / RollersConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1),
          1 / RollersConstants.GEAR_RATIO)

  private var appliedVoltage = 0.volts

  override fun updateInputs(inputs: IntakeRollersIO.RollerInputs) {
    rollerSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)
    inputs.leaderVelocity = rollerSim.angularVelocity.radians.perSecond
    inputs.leaderAcceleration = rollerSim.angularAcceleration.radians.perSecond.perSecond
    inputs.leaderAppliedVoltage = appliedVoltage
    inputs.leaderStatorCurrent = rollerSim.currentDraw.amps
    inputs.leaderSupplyCurrent = 0.amps
    inputs.leaderTemperature = 25.celsius
  }

  override fun setVoltage(voltage: ElectricalPotential) {
    rollerSim.setInputVoltage(voltage.inVolts)
    appliedVoltage = voltage
  }
}

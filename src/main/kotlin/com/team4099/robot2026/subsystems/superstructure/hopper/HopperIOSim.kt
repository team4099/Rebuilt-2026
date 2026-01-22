package com.team4099.robot2026.subsystems.superstructure.hopper
import com.team4099.robot2026.config.constants.Constants
import com.team4099.robot2026.subsystems.hopper.HopperIO
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import com.team4099.robot2026.config.constants.HopperConstants
import org.team4099.lib.units.derived.volts
import com.team4099.lib.math.clamp
import org.team4099.lib.units.base.amps
import org.team4099.lib.units.base.celsius
import org.team4099.lib.units.derived.ElectricalPotential

object HopperIOSim: HopperIO {
  private var appliedVoltage = 0.0.volts
  private val subsystemSim = FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getMotorName(numMotors),
      HopperConstants.MOMENT_OF_INERTIA.inKilogramMeterSquared,
      // reciprocal because in our code reductions are < 1, but in WPILib reductions are > 1
      1.0 / HopperConstants.GEAR_RATIO,
    ),
    DCMotor.getMotorName(numMotors),
  )
  override fun updateInputs(inputs: HopperIO.SubsystemIOInputs) {
    subsystemSim.update(Constants.Universal.LOOP_PERIOD_TIME.inSeconds)

    // general names are used for fields like position, velocity, etc.
    // in reality, these are probably things like angleRads or positionMeters
    // always cast to units!
    inputs.position = subsystemSim.position
    inputs.velocity = subsystemSim.velocity

    inputs.appliedVoltage = appliedVoltage
    inputs.supplyCurrent = subsystemSim.currentDrawAmps
    inputs.statorCurrent = 0.0.amps // not supplied by simulation
    inputs.temperature = 0.0.celsius // not supplied by simulation
  }
  override fun setVoltage(voltage: ElectricalPotential) {
    // we clamp the motor between the dedicated amount of voltage
    // for the subsystem
    val clampedVoltage =
      clamp(voltage, -HopperConstants.VOLTAGE_COMPENSATION, HopperConstants.VOLTAGE_COMPENSATION)

    subsystemSim.setInputVoltage(clampedVoltage)
    appliedVoltage = clampedVoltage
  }
}
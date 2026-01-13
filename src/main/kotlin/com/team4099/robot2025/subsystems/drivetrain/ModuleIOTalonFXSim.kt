// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.team4099.robot2025.subsystems.drivetrain

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.team4099.lib.phoenix6.PhoenixUtil
import com.team4099.robot2025.subsystems.drivetrain.generated.TunerConstants
import edu.wpi.first.units.Units.Radians
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.team4099.lib.units.derived.radians

class ModuleIOTalonFXSim(
  constants:
    SwerveModuleConstants<TalonFXConfiguration?, TalonFXConfiguration?, CANcoderConfiguration?>,
  val simulation: SwerveModuleSimulation
) : ModuleIOTalonFX(PhoenixUtil.regulateModuleConstantForSimulation(constants)) {
  init {
    simulation.useDriveMotorController(PhoenixUtil.TalonFXMotorControllerSim(driveTalon))
    simulation.useSteerMotorController(
      PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder)
    )

    //    super.positionTorqueCurrentRequest.withUpdateFreqHz(50.0).withUseTimesync(false)
    //    super.positionVoltageRequest.withUpdateFreqHz(50.0).withUseTimesync(false)
  }

  override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
    super.updateInputs(inputs)

    inputs.odometryTimestamps = PhoenixUtil.simulationOdometryTimeStamps
    inputs.odometryDrivePositions =
      simulation.cachedDriveWheelFinalPositions.map { it.`in`(Radians).radians }.toTypedArray()
    inputs.odometryTurnPositions =
      simulation.cachedSteerAbsolutePositions.map { it.radians.radians }.toTypedArray()
  }

  companion object {
    fun generateModules(simulation: SwerveDriveSimulation): Array<ModuleIO> {
      return arrayOf(
        ModuleIOTalonFXSim(TunerConstants.FrontLeft, simulation.modules[0]),
        ModuleIOTalonFXSim(TunerConstants.FrontRight, simulation.modules[1]),
        ModuleIOTalonFXSim(TunerConstants.BackLeft, simulation.modules[2]),
        ModuleIOTalonFXSim(TunerConstants.BackRight, simulation.modules[3])
      )
    }
  }
}

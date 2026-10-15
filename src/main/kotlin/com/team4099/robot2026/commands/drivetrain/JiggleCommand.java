package com.team4099.robot2026.commands.drivetrain;

import com.team4099.robot2026.subsystems.drivetrain.Drive;
import edu.wpi.first.units.measure.AngularVelocity;

class JiggleCommand(
        val drivetrain:Drive,
        val amplitude: AngularVelocity = 15.0.degrees.perSecond
        val frequencyHz: Double = 4.0
) : Command(){

}

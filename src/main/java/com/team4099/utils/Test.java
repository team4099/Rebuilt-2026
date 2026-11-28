package com.team4099.utils;

import org.wpilib.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

public class Test {
    public static void log(){
        Logger.recordOutput("Drivetrain/javaCooked", new Pose2d());
    }
}

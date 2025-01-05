// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import java.util.function.Supplier;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class PoseEstimatorConstants {

    public final static Vector<N3> ODOMETRY_DEVS = VecBuilder.fill(0.7, 0.7, 0.7);
    public final static Vector<N3> VISION_DEVS = VecBuilder.fill(0.7, 0.7, 0.1);

    public static Supplier<Boolean> ODOMETRY_UPDATE_CONSTRAINS = () -> DriverStation.isEnabled() && !DriverStation.isTest();

}

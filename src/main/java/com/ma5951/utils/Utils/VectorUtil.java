// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class VectorUtil {

    public static Translation2d getVectorFromSwerveState(SwerveModuleState state) {
        return new Translation2d(state.speedMetersPerSecond, state.angle);
    }

    

}

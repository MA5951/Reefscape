
package com.ma5951.utils.Utils;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtil {

    public static ChassisSpeeds FromFieldToRobot(ChassisSpeeds speeds, Rotation2d robotAngle) {
        var rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                .rotateBy(robotAngle.unaryMinus());

        speeds.vxMetersPerSecond = rotated.getX();
        speeds.vyMetersPerSecond = rotated.getY();
        return speeds;
    }

    public class ChassisSpeedsFieldUtil {

        private Supplier<Double> gyromMeasurment;
        private Supplier<Double> gyroOffset;

        public ChassisSpeedsFieldUtil(Supplier<Double> robotAngle, Supplier<Double> gyroOffset ) {
            gyromMeasurment = robotAngle;
            this.gyroOffset = gyroOffset;

        }
    }

}

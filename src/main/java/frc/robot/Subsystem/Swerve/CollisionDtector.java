
package frc.robot.Subsystem.Swerve;


import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystem.Swerve.Util.GyroData;

public class CollisionDtector {

    private Supplier<GyroData> gyroData;


    public CollisionDtector(Supplier<GyroData> GyroData) {
        gyroData = GyroData;
    }

    public double getForceVectorSize() {
        return Math.sqrt(Math.pow(getCollisionVector().getX(), 2) +
        Math.pow(getCollisionVector().getY(), 2));
    }

    public Translation2d getCollisionVector() {
        return new Translation2d(gyroData.get().getAccelX(), gyroData.get().getAccelY());
    }

}

 
package frc.robot.Subsystem.Swerve.Util;

public interface SwerveOdometry {

    void updateOdometry();

    double getLastSkid();

    double getLatCollision();
    
}

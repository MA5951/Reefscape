package frc.robot.Subsystem.Swerve.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Gyro {

    void reset();

    void updateOffset();

    double getPitch();

    double getRoll();

    double getYaw();

    double getAbsYaw();

    double getAccelX();

    double getAccelY();

    GyroData update(ChassisSpeeds robotSpeeds);

}


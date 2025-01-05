package frc.robot.Subsystem.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    //Return degrees
    double getAbsolutePosition();

    //Returns distance in meters
    double getDrivePosition();

    //Return position in degrees
    double getSteerPosition();

    //Return meters per second
    double getDriveVelocity();

    //Between 1 to -1
    void turningMotorSetPower(double power);

    //Between 1 to -1
    void driveMotorSetPower(double power);

    //Between 0 to 12
    void turningMotorSetVoltage(double volt);

    //Between 0 to 12
    void driveMotorSetVoltage(double volt);

    //Turns in degrees
    void turningUsingPID(double setPoint);

    //Drives in meters per second
    void driveUsingPID(double setPoint , double feedForward);

    void setNeutralModeDrive(Boolean isBrake);

    void setNeutralModeTurn(Boolean isBrake);

    void resetSteer();

    SwerveModuleData update();

    default SwerveModulePosition getPosition(SwerveModuleData moduleData) {
    return new SwerveModulePosition(moduleData.getDrivePosition() , new Rotation2d(moduleData.getSteerPosition()));
    }

    default SwerveModulePosition[] getModulePositions(SwerveModuleData moduleData) {
        int minOdometryPositions =
            Math.min(moduleData.getDrivePositionQueue().length, moduleData.getSteerPositionQueue().length);
        SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
        for (int i = 0; i < minOdometryPositions; i++) {
          positions[i] =
              new SwerveModulePosition(
                moduleData.getDrivePositionQueue()[i], moduleData.getSteerPositionQueue()[i]);
        }
        return positions;
      }

    default SwerveModuleState getState(SwerveModuleData moduleData) {
        return new SwerveModuleState(
                getDriveVelocity(), new Rotation2d(moduleData.getSteerPosition()) );
    }

    default void setDesiredState(SwerveModuleState desiredState) {
        driveUsingPID(desiredState.speedMetersPerSecond , 0);
        turningUsingPID(desiredState.angle.getRadians());
    }
}


package frc.robot.Subsystem.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroData {

    private double pitch;
    private double yaw;
    private double roll;
    private double veloYaw;
    private double veloPitch;
    private double veloRoll;
    private double accelX;
    private double accelY;
    private double absoluteYaw; // 0 is red
    private Rotation2d[] yawPositionQueue;

    public GyroData() {
        this(0, 0, 0, 0, 0, 0, 0, 0, 0, new Rotation2d[0]);
    }

    public GyroData(double Pitch, double Yaw, double Roll, double VeloYaw, double VeloPitch,
    double VeloRoll, double AccelX, double AccelY, double AbsoluteYaw , Rotation2d[] YawPositionQueue) {
        pitch = Pitch;
        yaw = Yaw;
        roll = Roll;
        veloYaw = VeloYaw;
        veloPitch = VeloPitch;
        veloRoll = VeloRoll;
        accelX = AccelX;
        accelY = AccelY;
        absoluteYaw = AbsoluteYaw;
        yawPositionQueue = YawPositionQueue;
    }

    public void updateData(double Pitch, double Yaw, double Roll, double VeloYaw, double VeloPitch,
    double VeloRoll, double AccelX, double AccelY, double AbsoluteYaw , Rotation2d[] YawPositionQueue) {
        pitch = Pitch;
        yaw = Yaw;
        roll = Roll;
        veloYaw = VeloYaw;
        veloPitch = VeloPitch;
        veloRoll = VeloRoll;
        accelX = AccelX;
        accelY = AccelY;
        absoluteYaw = AbsoluteYaw;
        yawPositionQueue = YawPositionQueue;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }

    public double getRoll() {
        return roll;
    }

    public double getVeloYaw() {
        return veloYaw;
    }

    public double getVeloPitch() {
        return veloPitch;
    }

    public double getVeloRoll() {
        return veloRoll;
    }

    public double getAccelX() {
        return accelX;
    }

    public double getAccelY() {
        return accelY;
    }

    public double getAbsoluteYaw() {
        return absoluteYaw;
    }

    public Rotation2d[] getYawPositionQueue() {
        return yawPositionQueue;
    }

}

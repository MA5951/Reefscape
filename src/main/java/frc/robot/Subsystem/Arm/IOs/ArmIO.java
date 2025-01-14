package frc.robot.Subsystem.Arm.IOs;

public interface ArmIO {

    double getAbsolutePosition();

    double getCurrent();

    double getPosition();

    double getVelocity();

    double getAppliedVolts();

    double getError();

    double getSetPoint();

    void resetPosition(double newPose);

    void updatePID(double Kp, double Ki, double Kd);

    void setNeutralMode(boolean isBrake);

    void setVoltage(double volt);

    void setAngle(double angle);

    void updatePeriodic();
}

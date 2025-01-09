
package frc.robot.Subsystem.Elevator.IOs;

public interface ElevatorIO {

    boolean getLimitSwitch();

    double getCurrent();

    double getPosition();

    double getVelocity();

    double getAppliedVolts();

    double getError();

    double getSetPoint();

    void resetPosition(double newPose);

    void updatePID(double Kp , double Ki , double Kd);

    void setNutralMode(boolean isBrake);

    void setVoltage(double volt);
    
    void setHight(double angle);

    void updatePeriodic();
}

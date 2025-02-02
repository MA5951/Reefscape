
package frc.robot.Subsystem.Climb.IOs;

public interface ClimbIO {

    void setVoltage(double volt);

    void setMasterNutralMode(boolean isBreak);

    void setServo(double position);

    double getServoPose();

    double getMasterVelocity();

    double getMasterCurrent();

    double getMasterAppliedVolts();

    double getPosition();

    Boolean getLimitSensor();

    void updatePeriodic();
    
} 

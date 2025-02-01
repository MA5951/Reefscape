
package frc.robot.Subsystem.Climb.IOs;

public interface ClimbIO {

    void setVoltage(double volt);

    void setMasterNutralMode(boolean isBreak);

    double getMasterVelocity();

    double getMasterCurrent();

    double getMasterAppliedVolts();

    Boolean getLimitSensor();

    void updatePeriodic();
    
} 

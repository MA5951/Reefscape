
package frc.robot.Subsystem.climb.IOs;

public interface ClimbIO {

    void setVoltage(double volt);

    void setMasterNutralMode(boolean isBreak);

    double getMasterVelocity();

    double getMasterCurrent();

    double getMasterAppliedVolts();

    Boolean getFirstSensor();

    Boolean getSecondSensor();

    void updatePeriodic();
    
} 

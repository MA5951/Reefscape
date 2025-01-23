
package frc.robot.Subsystem.climb.IOs;

public interface ClimbIO {

    void setVoltage(double volt);

    void setRightNutralMode(boolean isBreak);

    void setLeftNutralMode(boolean isBreak);

    double getRightVelocity();

    double getRightCurrent();

    double getRightAppliedVolts();

    double getLeftVelocity();

    double getLeftCurrent();

    double getLeftAppliedVolts();

    Boolean getFirstSensor();

    Boolean getSecondSensor();

    void updatePeriodic();
    
} 

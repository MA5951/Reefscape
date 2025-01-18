
package frc.robot.Subsystem.Intake.IOs;

public interface IntakeIO {

    boolean getFrontSensor();

    boolean getRearSensor();

    double getIntendedVoltage();

    double getCurrent();

    double getPosition();

    double getVelocity();

    double getAppliedVolts();

    void setNutralMode(boolean isBrake);

    void setVoltage(double volt);

    void updatePeriodic();
    
}

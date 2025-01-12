package frc.robot.Subsystem.Arm;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Arm.IOs.ArmIO;

public class Arm extends StateControlledSubsystem {
    private static Arm arm;

    private ArmIO armIO = ArmConstants.getArmIO();

    public Arm() {
        super(ArmConstants.SUBSYSTEM_STATES, "Arm");
    }

    public double getAbsolutePosition() {
        return armIO.getAbsolutePosition();
    }

    public double getCurrent() {
        return armIO.getCurrent();
    }

    public double getPosition() {
        return armIO.getPosition();
    }

    public double getVelocity() {
        return armIO.getVelocity();
    }

    public double getAppliedVolts() {
        return armIO.getAppliedVolts();
    }


    public double getSetPoint() {
        return armIO.getSetPoint();
    }

    public void resetPose(double newPose) {
        armIO.resetPosition(newPose);
    }


    public void setNeutralMode(boolean isBrake) {
        armIO.setNeutralMode(isBrake);
    }

    public void setVoltage(double volt) {
        armIO.setVoltage(volt);
    }

    public void setAngle(double angle) {
        armIO.setAngle(angle);
    }

    public boolean atPoint() {
        return armIO.getError() <= ArmConstants.TOLERANCE;
    }

    public boolean limitsCANMOVE() {
        return getSetPoint() < ArmConstants.MAX_ANGLE && getSetPoint() > ArmConstants.MIN_ANGLE;
    }

    @Override
    public boolean canMove() {
        return true;
    }

    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    @Override
    public void periodic() {
        armIO.updatePeriodic();
    }
}

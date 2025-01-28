// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;

public class Elevator extends StateControlledSubsystem {
  private static Elevator elevator;

  private ElevatorIO elevatorIO = ElevatorConstants.getElevatorIO();
  private LoggedBool atPointLog;
  private Debouncer atPointDebouncer = new Debouncer(RobotConstants.kDELTA_TIME * 2);

  private Elevator() { 
    super(ElevatorConstants.SUBSYSTEM_STATES, "Elevator");
    atPointLog = new LoggedBool("/Subsystems/Elevator/AtPoint");
  }

  public double getFeedForwardVoltage() {
    return ElevatorConstants.FEED_FORWARD;
  }

  public boolean getLimitSwitch() {
    return elevatorIO.getLimitSwitch();
  }

  public void resetPose(double hight) {
    elevatorIO.resetPosition(hight);
  }

  public double getHight() {
    return elevatorIO.getPosition();
  }

  public double getVelocity() {
    return elevatorIO.getVelocity();
  }

  public double getAppliedVolts() {
    return elevatorIO.getAppliedVolts();
  }

  public double getCurrent() {
    return elevatorIO.getCurrent();
  }

  public boolean atPoint() {
    return atPointDebouncer.calculate(Math.abs(elevatorIO.getError()) <= ElevatorConstants.TOLORANCE);
  }

  public double getSetPoint() {
    return elevatorIO.getSetPoint();
  }

  public void setNutralMode(boolean isBrake) {
    elevatorIO.setNutralMode(isBrake);
  }

  public void setVoltage(double volt) {
    elevatorIO.setVoltage(volt);
  }

  public void setHight(double hight) {
    elevatorIO.setHight(hight);
  }

  @Override
  public boolean canMove() {
    return getSystemFunctionState() == StatesConstants.MANUEL
        || getHight() <= ElevatorConstants.MAX_HIGHT && getHight() >= ElevatorConstants.MIN_HIGHT &&
            Math.abs(getCurrent()) <= ElevatorConstants.CAN_MOVE_CURRENT_LIMIT;
  }

  public static Elevator getInstance() {
    if (elevator == null) {
      elevator = new Elevator();
    }
    return elevator;
  }

  @Override
  public void periodic() {
    super.periodic();
    elevatorIO.updatePeriodic();
    atPointLog.update(atPoint());
  }
}

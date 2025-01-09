// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;

public class Elevator extends StateControlledSubsystem {
  private static Elevator elevator;

  private ElevatorIO elevatorIO = ElevatorConstants.getElevatorIO();

  public Elevator() {
    super(null, "Elevator");
  }

  public void resetPose() {

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

  public boolean atPoint() {
    return elevatorIO.getError() <= ElevatorConstants.TOLORANCE; 
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
  public void periodic() {
    elevatorIO.updatePeriodic();
  }
}

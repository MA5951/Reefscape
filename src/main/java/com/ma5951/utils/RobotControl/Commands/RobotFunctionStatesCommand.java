// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.RobotControl.Commands;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj.DriverStation;


public class RobotFunctionStatesCommand extends CanMoveCommand {
  /** Creates a new RobotFunctionStatesCommand. */
  private StateControlledSubsystem subsystem;
  
  public RobotFunctionStatesCommand(StateControlledSubsystem subsystem) {
    super(subsystem);
    this.subsystem = super.subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void CAN_MOVE() {
    super.CAN_MOVE();
    switch (getRobotFunctionStateAsInt()) {
                            case 0:
                              switch (subsystem.getSystemFunctionState().getName()) {
                                case "AUTOMATIC":
                                    AutomaticLoop();
                                    break;
                            
                                case "MANUEL":
                                    ManuelLoop();
                                    break;
                            }
                                break;
                            case 1:
                                AutoLoop();
                                break;
                            case 2:
                                TestLoop();
                                break;
    }
  }

  private int getRobotFunctionStateAsInt() {
    if (DriverStation.isTeleop()) {
      return 0;
  } else if (DriverStation.isAutonomous()) {
      return 1;
  } else {
      return 0;
  }
  }

  @Override
  public void CANT_MOVE() {
      super.CANT_MOVE();
  }

  public void AutomaticLoop() {

  }

  public void ManuelLoop() {

  }

  public void AutoLoop() {
    
  }

  public void TestLoop() {
    
  }


  
}

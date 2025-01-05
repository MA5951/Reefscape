// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.RobotControl.Commands;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;


public class CanMoveCommand extends DisableEnableCommand {
  /** Creates a new CanMoveCommand. */
  protected StateControlledSubsystem subsystem;

  public CanMoveCommand(StateControlledSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public int convertCanMoveToInt(boolean canMove) {
    if (canMove) {
        return 1;
    } else {
        return 0;
    }
  }

  @Override
  public void ENABLE_LOOP() {
      super.ENABLE_LOOP();

      switch (convertCanMoveToInt(subsystem.canMove())) {
                    case 1:
                        CAN_MOVE();
                        break;
                
                    default:
                        CANT_MOVE();
                        break;
                }
    }

  @Override
  public void DISABLE_LOOP() {
      super.DISABLE_LOOP();
  }


  public void CAN_MOVE() {

  }

  public void CANT_MOVE() {

  }
}

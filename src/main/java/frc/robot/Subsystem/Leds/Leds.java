// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Leds;

import com.ma5951.utils.Leds.LEDBase;
import com.ma5951.utils.RobotControl.DeafultRobotContainer;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;

public class Leds extends LEDBase {
  private static Leds leds;

  private Leds() {
    super(PortMap.Led.ledPort, LedsConstants.LED_LENGTH , RobotConstants.USE_LEDS);
  }

  @Override
  public void runTeleopAnimation() {
    if (RobotContainer.currentRobotState == RobotConstants.INTAKE && SuperStructure.hasGamePiece()) {
      blinkColorPattern(0.5, LedsConstants.BLACK, LedsConstants.GREEN);
    } else if (RobotContainer.currentRobotState == RobotConstants.INTAKE ) {
      blinkColorPattern(1, LedsConstants.BLACK, LedsConstants.BLUE);
    } else if (RobotContainer.currentRobotState == RobotConstants.SCORING && RobotContainer.intake.getAppliedVolts() < 0) {
      blinkColorPattern(0.5, LedsConstants.BLACK, LedsConstants.GREEN);
    } else if (RobotContainer.currentRobotState == RobotConstants.SCORING || RobotContainer.currentRobotState == RobotConstants.BALLREMOVING) {
      smoothWaveColorPattern(2, 1, 1,  new Color [] {LedsConstants.CONE_YELLOW, LedsConstants.CUBE_PURPLE, LedsConstants.CYAN});
    } else {
      
    }
  }

  @Override
  public void runAutoAnimation() {
    smoothWaveColorPattern(2, 1, 1,  new Color [] {LedsConstants.CONE_YELLOW, LedsConstants.CUBE_PURPLE, LedsConstants.CYAN});
    
  }

  @Override
  public void runDisableAnimation() {
    if (!DriverStation.isDSAttached()) {
      smoothWaveColorPattern(2, 1, 1,  new Color [] {LedsConstants.BLACK, LedsConstants.PURPLE});
    }else {
      if (RobotContainer.alliance != null) {
        if (RobotContainer.alliance == Alliance.Red) {
          smoothWaveColorPattern(2, 1, 1,  new Color [] {LedsConstants.BLACK, LedsConstants.RED});
        } else if (RobotContainer.alliance == Alliance.Blue) {
          smoothWaveColorPattern(2, 1, 1,  new Color [] {LedsConstants.BLACK, LedsConstants.BLUE});
        } 
      }
    }
  }

  public static Leds getInstance() {
    if (leds == null) {
      leds = new Leds();
    }
    return leds;
  }

}
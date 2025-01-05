// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.DashBoard;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoOption {

    private Command autoCommand;
    private String name;
    private boolean isPathPlanner = false;
    private String pathPlannerName;
    private Pose2d startPose;

    //Use this to create auto option that was created with path planner
    public AutoOption(String optionName , String PathPlannerAutoName , Pose2d StartingPose) {
        autoCommand = AutoBuilder.buildAuto(PathPlannerAutoName);
        name = optionName;
        pathPlannerName = PathPlannerAutoName;
        isPathPlanner = true;
        startPose = StartingPose;
    }

    //Use this to create auto option that was created with regular command 
    public AutoOption(String optionName,Command command, Pose2d StartingPose) {
        autoCommand = command;
        name = optionName;
        startPose = StartingPose;
    }

    public String getName() {
        return name;
    }

    public boolean isPathPlannerAuto() {
        return isPathPlanner;
    }

    public String getPathPlannerAutoName() {
        return pathPlannerName;
    }

    public Command getCommand() {
        return autoCommand;
    }

    public Pose2d getStartPose() {
        return startPose;
    }
}

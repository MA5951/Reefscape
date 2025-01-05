// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.DashBoard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import org.json.simple.parser.ParseException;

import com.ma5951.utils.Utils.DriverStationUtil;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

@SuppressWarnings("unused")
public class AutoSelector {
    private SendableChooser<edu.wpi.first.wpilibj2.command.Command> commandChooser;
    private AutoOption[] autoOptions;

    double[] lastarr = new double[0];
    private String lastPublishedAuto = "0";
    private Field2d field = new Field2d();
    private List<Pose2d> posesList = new ArrayList<>();
    private DoubleArrayPublisher pathPub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/Dash/path")
            .publish();
    private static final HashMap<Command, AutoOption> autoOptionByCommand = new HashMap<>();
    private boolean preVizualizAuto;
    private PathPlannerPath[] pathsArry;
    private Supplier<Pose2d> poseSupplier;
    private String Auto;

    public AutoSelector(Supplier<Pose2d> robotPoseSupplier) {
        commandChooser = new SendableChooser<>();
        Shuffleboard.getTab("Auto").add("AutoSelector", commandChooser);
        SmartDashboard.putData(field);
        poseSupplier = robotPoseSupplier;

    }

    public void setAutoOptions(AutoOption[] options, boolean preViz) {
        preVizualizAuto = preViz;
        autoOptions = options;
        for (AutoOption autoOption : autoOptions) {
            commandChooser.addOption(autoOption.getName(), autoOption.getCommand());
            autoOptionByCommand.put(autoOption.getCommand(), autoOption);
        }

        commandChooser.setDefaultOption(autoOptions[0].getName(), autoOptions[0].getCommand());

    }

    public Command getSelectedAutoCommand() {
        return commandChooser.getSelected();
    }

    public AutoOption getSelectedAuto() {
        if (autoOptionByCommand.get(getSelectedAutoCommand()) != null) {
            return autoOptionByCommand.get(getSelectedAutoCommand());
        }

        return new AutoOption("", new InstantCommand(), new Pose2d());
    }

    public String getAutonomousName() {
        return getSelectedAuto().getName();
    }

    public boolean getIsPathPLannerAuto() {
        return getSelectedAuto().isPathPlannerAuto();
    }

    public AutoOption getCurrentSelectedAutoOption() {
        return getSelectedAuto();
    }

    public Pose2d getSelectedAutoStartingPose() {
        return getSelectedAuto().getStartPose();
    }

    // Previz stuff
    public void clearCurrentPath() {
        pathPub.set(new double[0]);
        lastarr = new double[0];
        posesList = new ArrayList<>();
        field.getObject("Auto").setPoses(posesList);
    }

    public void setCurrentPath(PathPlannerPath path) {
        double[] arr = new double[path.numPoints() * 3];
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                path = path.flipPath();
            }
        }
        int ndx = 0;
        for (PathPoint p : path.getAllPathPoints()) {
            Translation2d pos = p.position;
            arr[ndx] = pos.getX();
            arr[ndx + 1] = pos.getY();
            // Just add 0 as a heading since it's not needed for displaying a path
            arr[ndx + 2] = 0.0;
            ndx += 3;
        }

        Stream<Double> stream1 = Arrays.stream(lastarr).boxed();
        Stream<Double> stream2 = DoubleStream.of(arr).boxed();

        Stream<Double> combinedStream = Stream.concat(stream1, stream2);
        double[] combinedArray = combinedStream.mapToDouble(Double::doubleValue).toArray();
        lastarr = combinedArray;
        pathPub.set(lastarr);
    }

    public void updateViz() {
        field.setRobotPose(poseSupplier.get());
        if (preVizualizAuto) {
            if (getSelectedAuto().isPathPlannerAuto()) {
                if (lastPublishedAuto != getSelectedAuto().getPathPlannerAutoName()) {
                    clearCurrentPath();
                    Auto = getSelectedAuto().getPathPlannerAutoName();
                    try {
                        pathsArry = PathPlannerAuto.getPathGroupFromAutoFile(Auto).toArray(new PathPlannerPath[0]);
                    } catch (IOException | ParseException e) {
                        e.printStackTrace();
                    }
                    for (PathPlannerPath path : pathsArry) {
                        if (DriverStationUtil.getAlliance() == Alliance.Red) {
                            path.flipPath();
                        }
                        setCurrentPath(path);
                    }

                    lastPublishedAuto = getSelectedAuto().getPathPlannerAutoName();
                    for (int i = 0; i < lastarr.length; i += 3) {
                        double x = lastarr[i];
                        double y = lastarr[i + 1];
                        double rotation = lastarr[i + 2];
                        posesList.add(new Pose2d(x, y, new Rotation2d(rotation)));
                    }
                    field.getObject("Auto").setPoses(posesList);
                }
            } else {
                clearCurrentPath();
            }

        }
    }
}

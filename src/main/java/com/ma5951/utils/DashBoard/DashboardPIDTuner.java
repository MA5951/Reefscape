
package com.ma5951.utils.DashBoard;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Function;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

@SuppressWarnings("unused")
public class DashboardPIDTuner implements Sendable {

    private String name;
    private Consumer<Double> kPConsumer;
    private Consumer<Double> kIConsumer;
    private Consumer<Double> kDConsumer;

    public DashboardPIDTuner(String name , Function<Double , Function<Double , Consumer<Double>>> it) {
        this.name = name;

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(name + "PIDCTuner");
        builder.addDoubleProperty("kP", () -> 1, null);
        builder.addDoubleProperty("kI", () -> 2, null);
        builder.addDoubleProperty("kD", () -> 3, null);
        builder.addDoubleProperty("Set Point", null, null);
    }
}
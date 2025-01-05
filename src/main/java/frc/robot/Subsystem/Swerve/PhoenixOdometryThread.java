// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Subsystem.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Swerve.IOs.SwerveThreadOdometry;

import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> queues = new ArrayList<>();
  
  private boolean isCANFD = false;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
    start();
  }

  public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    SwerveThreadOdometry.odometryLock.lock();
    try {
      isCANFD = PortMap.Swerve.SwervBus.isNetworkFD();
      BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
      System.arraycopy(signals, 0, newSignals, 0, signals.length);
      newSignals[signals.length] = signal;
      signals = newSignals;
      queues.add(queue);
    } finally {
      signalsLock.unlock();
      SwerveThreadOdometry.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD) {
          BaseStatusSignal.waitForAll(RobotConstants.kDELTA_TIME, signals);
        } else {
          Thread.sleep((long) (1000.0 / SwerveConstants.ODOMETRY_UPDATE_RATE));
          if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }
      double fpgaTimestamp = Timer.getFPGATimestamp() / 1.0e6;

      // Save new data to queues
      SwerveThreadOdometry.odometryLock.lock();
      try {
        for (int i = 0; i < signals.length; i++) {
          queues.get(i).offer(signals[i].getValueAsDouble());
        }
        SwerveThreadOdometry.timestampQueue.offer(fpgaTimestamp);
      } finally {
        SwerveThreadOdometry.odometryLock.unlock();
      }
    }
  }
}

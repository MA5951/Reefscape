
package frc.robot.Subsystem.climb.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimbIOSim extends ClimbIOReal{

    private TalonFXMotorSim rightMotorSim;
    private TalonFXMotorSim leftMotorSim;

    public ClimbIOSim() {
        super();
        rightMotorSim = new TalonFXMotorSim(rightMotor, rightConfig, DCMotor.getKrakenX60(1), 0, false);//TODO:I dont sure
        leftMotorSim = new TalonFXMotorSim(leftMotor, leftConfig, DCMotor.getKrakenX60(1), 0, false);//TODO:I dont sure
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        rightMotorSim.updateSim();
        leftMotorSim.updateSim();
    }
}

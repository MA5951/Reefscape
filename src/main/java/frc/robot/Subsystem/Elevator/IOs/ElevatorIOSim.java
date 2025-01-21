
package frc.robot.Subsystem.Elevator.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends ElevatorIOReal{

    private TalonFXMotorSim elevatorMotorSim;
    private ElevatorSim elevatorSim;

    public ElevatorIOSim() {
        super();
        elevatorSim = new ElevatorSim(null, getVelocity(), getSetPoint(), getPosition(), getError(), getCurrent(), getLimitSwitch(), getAppliedVolts(), null)
        elevatorMotorSim = new TalonFXMotorSim(masterMotor, masterConfig, DCMotor.getKrakenX60(1), 0.1, false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        elevatorMotorSim.updateSim();
    }
}

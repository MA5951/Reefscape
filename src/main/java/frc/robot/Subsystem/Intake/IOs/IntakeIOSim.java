
package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim extends IntakeIOReal {

    private TalonFXMotorSim intakeMotorSim;

    public IntakeIOSim() {
        super();
        intakeMotorSim = new TalonFXMotorSim(intakeMotor, motorConfig, DCMotor.getFalcon500(1), 0.05, false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        intakeMotorSim.updateSim();
    } 
}


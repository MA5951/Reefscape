
package frc.robot.Subsystem.Intake.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSim extends IntakeIOReal {

    private TalonFXMotorSim intakeMotorSim;
    private DigitalInput frontSensorSim;
    private DigitalInput rearSensorSim;

    public IntakeIOSim() {
        super();
        intakeMotorSim = new TalonFXMotorSim(intakeMotor, motorConfig, DCMotor.getFalcon500(1), 0.05, false);
        frontSensorSim = new DigitalInput(0);
        rearSensorSim = new DigitalInput(0);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        intakeMotorSim.updateSim();
        intakeMotorSim.updateMotorLimits(frontSensorSim.get() , rearSensorSim.get() );
    } 
}


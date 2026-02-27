package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    double power = 0;
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

        public Intake(IntakeIO io) {
        this.io = io;
    }
    

    public Command intakeRunRollers(){
                return this.runOnce(
            () -> {
                // Set power to full (1)
                this.power = 1;

                // Use power to start the KickerIO Hardware motor
                this.io.rollersRunVolts(this.power);
            }
        );
    }

    public Command intakeStopRollers(){
                    return this.runOnce(
            () -> {
                this.io.rollersStop();
            }
        ); 
    }

        @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update inputs object with the current status of the KickerIO hardware
        // and then write values to the Log
        this.io.updateInputs(inputs);
        Logger.processInputs("Kicker Subsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

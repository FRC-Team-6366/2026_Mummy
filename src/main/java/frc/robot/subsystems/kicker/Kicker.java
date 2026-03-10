package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Kicker subsystem class for the FRC 2026 robot. This class handles the
 * hardware and interactions for moving fuel from the indexer subsystem
 * to the shooter subsystem
 * @author Hayden H
 * @author Will E (Mentor)
 */
public class Kicker extends SubsystemBase {
    /**
     * Double value to hold the current power setting for the kicker subsystem
     * motor. This is used for the increment/decrement commands mainly
     */
    double power = 0;
    KickerIO io;
    
    /**
     * IOInputs object for holding current values for the devices of the 
     * Kicker Subsystem. These values are updated every loop and written
     * to the Log for reviewing or replaying
     */
    KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    /**
     * Creates Kicker subsystem by using the supplied KickerIO object that represents
     * the underlying hardware for the kicker subsystem
     * <p>
     * Example use:
     * <pre>{@code Kicker kicker = new Kicker(new KickerIOTalonFX());}</pre>
     * @param io Hardware object that implements the KickerIO interface class
     * 
     */
    public Kicker(KickerIO io) {
        this.io = io;
    }

    /**
     * Returns the command for the kicker to turn on, thus moving
     * fuel into the shooter
     * <p>
     * Example use:
     * <pre>{@code controller.a().whileTrue(kicker.turnOnKicker());}</pre>
     * @return Command for turning on the kicker subsystem
     */
    public Command runKicker() {
        return this.run(
            () -> {
                // Set power to full (1)
                this.power = 1;

                // Use power to start the KickerIO Hardware motor
                this.io.setKickPower(this.power);
            }
        ).withName("runKicker()");
    }

    /**
     * Returns the command for the kicker to turn off, thus stopping
     * fuel from being sent into the shooter
     * <p>
     * Example use:
     * <pre>{@code controller.b().whileTrue(kicker.turnOffKicker());}</pre>
     * @return Command for turning off the kicker subsystem
     */
    public Command stopKicker() {
        return this.runOnce(
            () -> {
                // Set power to 0
                this.power = 0;

                // Use power to stop the KickerIO Hardware motor
                this.io.setKickPower(this.power);
            }
        ).withName("stopKicker()");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update inputs object with the current status of the KickerIO hardware
        // and then write values to the Log
        this.io.updateInputs(inputs);
        Logger.processInputs("Kicker Subsystem", inputs);
        Logger.recordOutput("KickerSubsystem/DefaultCommand", this.getDefaultCommand()!=null ? this.getDefaultCommand().getName() : "N/A");
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

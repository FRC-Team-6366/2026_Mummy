package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
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
     * Increases the kicker subsystem's output by 0.3 to a maximum power
     * of 1.
     * <p>
     * Example use:
     * <pre>{@code controller.leftTrigger().whileTrue(kicker.kickIncrement());}</pre>
     * @return Command to increase the kicker subsystem output by 0.3
     */
    public Command kickIncrement() {
        return runOnce(
            () -> {
                // Clamp method returns either power, or the max or min value
                // This ensures that power will never be greater than 1
                this.power = MathUtil.clamp(this.power += 0.3, 0, 1);
                
                // Set the power of the KickerIO hardware
                this.io.setKickPower(power);
            }
        );
    }

    /**
     * Decreases the kicker subsystem's output by 0.3 down to a minimum power
     * of 0.
     * <p>
     * Example use:
     * <pre>{@code controller.rightTrigger().whileTrue(kicker.kickDeccrement());}</pre>
     * @return Command to decrease the kicker subsystem output by 0.3
     */
    public Command kickDecrement() {
        return runOnce(
            () -> {
                // Clamp method returns either power, or the max or min value
                // This ensures that power will never be less than 0
                this.power = MathUtil.clamp(this.power -= 0.3, 0, 1);
                
                // Set the power of the KickerIO hardware
                this.io.setKickPower(power);
            }
        );
    }

    /**
     * Returns the command for the kicker to turn on, thus moving
     * fuel into the shooter
     * <p>
     * Example use:
     * <pre>{@code controller.a().whileTrue(kicker.turnOnKicker());}</pre>
     * @return Command for turning on the kicker subsystem
     */
    public Command turnOnKicker() {
        return this.runOnce(
            () -> {
                // Set power to full (1)
                this.power = 1;

                // Use power to start the KickerIO Hardware motor
                this.io.setKickPower(this.power);
            }
        );
    }

    /**
     * Returns the command for the kicker to turn off, thus stopping
     * fuel from being sent into the shooter
     * <p>
     * Example use:
     * <pre>{@code controller.b().whileTrue(kicker.turnOffKicker());}</pre>
     * @return Command for turning off the kicker subsystem
     */
    public Command turnOffKicker() {
        return this.runOnce(
            () -> {
                // Set power to 0
                this.power = 0;

                // Use power to stop the KickerIO Hardware motor
                this.io.setKickPower(this.power);
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

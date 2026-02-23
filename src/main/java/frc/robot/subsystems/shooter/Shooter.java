package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/**
 *Big H here- this Substemy is for the Shooter class for the 2026 Mummybot. This class
 * handles the interactions for for shooting the subsystem, and should have the could for the 
 * shooting hood as well.
 * @author Hayden
 * @author Will 
 */
public class Shooter extends SubsystemBase{
    /**
     * Double value used for the current power setting in
     *  increment and decrement commmands in the shooter subsystem.
     */
    private double velocityRPS = 0;
    ShooterIO shooterIO;

    /**
     * IOInputs object that holds and updates values of the devices in the 
     * shooter system. It get logged and updated every loop.
     */
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /**
     * Creates the subsystem using the shooterIO object and represents
     * the hardware for the shooter subsystem
     * <p>
     * Example use:
     * <pre>{@code Shooter shooter = new Shooter(new ShooterIOTalonFX());}</pre>
     * @param io Hardware object that implements the ShooterIO interface class
     * 
     */
    public Shooter(ShooterIO io ){
        this.shooterIO = io;
    }

     /**
     * Increases the shooter subsystem's output by 0.3 to a maximum power
     * of 1.
     * <p>
     * Example use:
     * <pre>{@code controller.leftTrigger().whileTrue(shooter.shooterIncrement());}</pre>
     * @return Command to increase the shooter subsystem output by 0.3
     */
    public Command shooterIncrements() {
        return runOnce(
            () -> {                
                this.velocityRPS += 5;
                // Set the power of the ShooterIO hardware
                this.shooterIO.setShooterVelocityRPS(this.velocityRPS);
            }
        );
    }

     /**
     * Decreases the shooter subsystem's output by 0.3 down to a minimum power
     * of 0.
     * <p>
     * Example use:
     * <pre>{@code controller.rightTrigger().whileTrue(shooter.shooterDeccrements());}</pre>
     * @return Command to decrease the shooter subsystem output by 0.3
     */
    public Command shooterDecrements() {
        return runOnce(
            () -> {
                // Clamp method returns either power, or the max or min value
                // This ensures that power will never be greater than 1
                this.velocityRPS -= 5;
                
                // Set the power of the KickerIO hardware
                this.shooterIO.setShooterVelocityRPS(velocityRPS);
            }
        );
    }

    /**
     * Stops the shooter, setting the power to 0.
     * <p>
     * Example use:
     * <pre>{@code controller.b().whileTrue(shppter.turnOffShooter());}</pre>
     * @return Command for turning off the shooter subsystem
     */
    public Command turnOffShooter() {
        return this.runOnce(
            () -> {
                // Set power to 0
                this.velocityRPS = 0;

                // Use power to stop the ShooterIO Hardware motor
                this.shooterIO.setShooterVelocityRPS(this.velocityRPS);
            }
        );
    }


    /**
     * Sets the shooter for shooting at close targets
     * @return Command to set shooter for close shooting
     */
    public Command setShooterVelocityLow(){
        return this.runOnce(
            () -> {
                this.shooterIO.setShooterVelocityFeetPerSecond(Constants.ShooterConstants.shooterPosition1VelocityFPS);
            }
        );
    }

    /**
     * Sets the shooter for shooting at medium distance targets
     * @return Command to set shooter for medium shooting
     */
    public Command setShooterVelocityMedium(){
        return this.runOnce(
            () -> {
                this.shooterIO.setShooterVelocityFeetPerSecond(Constants.ShooterConstants.shooterPosition2VelocityFPS);
            }
        );
    }

    /**
     * Sets the shooter for shooting at far away targets
     * @return Command to set shooter for far shooting
     */
    public Command setShooterVelocityHigh(){
        return this.runOnce(
            () -> {
                this.shooterIO.setShooterVelocityFeetPerSecond(Constants.ShooterConstants.shooterPosition3VelocityFPS);
            }
        );
    }

    /**
     * Returns whether the shooter is at its set point velocity, given a percent of tolerence
     * specified in the ShooterIO hardware class
     * @return BooleanSupplier: True hood is at its setpoint, false otherwise
     */
    public BooleanSupplier shooterAtVelocitySetPoint() {
        return () -> shooterIO.shooterAtVelocitySetPoint();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update inputs object with the current status of the ShooterIO hardware
        // and then write values to the Log
        this.shooterIO.updateInputs(inputs);
        Logger.processInputs("ShooterSubsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

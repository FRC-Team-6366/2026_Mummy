package frc.robot.subsystems.shooter.hood;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    HoodIO hoodIO;
    HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.hoodIO = io;
    }

    /**
     * Retracts the hood to its starting position
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood to starting position
     */
    public Command retractHood() {

        return this.hoodToAngle(0);
    }
    
    /**
     * Sets the hood for shooting at specified angle
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @param angle Angle in degrees
     * @return Command to set hood at angle for shooting
     */
    public Command hoodToAngle(double angle) {
        return this.runOnce(
            () -> {
                this.hoodIO.hoodToAngle(angle);
            });
    }

    /**
     * Sets the hood for shooting at hanging station
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for close shooting
     */
    public Command hoodToAnglePosition1(){
        return this.hoodToAngle(Constants.ShooterConstants.hoodPosition1Angle);
    }

    /**
     * Sets the hood for shooting at trench wall
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for medium shooting
     */
    public Command hoodToAnglePosition2(){
        return this.hoodToAngle(Constants.ShooterConstants.hoodPosition2Angle);
    }

    /**
     * Sets the hood for shooting at human player station
     * <p>
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to set hood for far shooting
     */
    public Command hoodToAnglePosition3(){
        return this.hoodToAngle(Constants.ShooterConstants.hoodPosition3Angle);
    }

    /**
     * Returns whether the hood is at its set point distance, given a percent of tolerence
     * specified in the HoodIO hardware class
     * @return BooleanSupplier: True hood is at its setpoint, false otherwise
     */
    public BooleanSupplier hoodAtPositionSetpoint() {
        return () -> this.hoodIO.hoodAtPositionSetpoint();
    }

    @Override
    public void periodic() {
        this.hoodIO.updateInputs(inputs);
        Logger.processInputs("HoodSubsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}

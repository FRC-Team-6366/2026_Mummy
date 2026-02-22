package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    double setPointHoodDegree;
    HoodIO hoodIO;
    HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io) {
        this.hoodIO = io;
    }

    public Command hoodToAngle(double angle) {
        this.setPointHoodDegree = angle;
        Logger.recordOutput("HoodSubsystem/Setpoints/setpoint", this.setPointHoodDegree);
        return this.runOnce(
                () -> {
                    this.hoodIO.hoodToAngle(this.setPointHoodDegree);
                });
    }

    /**
     * Sets the hood to rotate to position of 0 rotations (0 degrees)
     * from the hoods starting position when the robot was turned on.
     * 
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to move hood to position 0
     */
    public Command hoodToPosition0(){
        return this.runOnce(
            () -> {
                this.hoodIO.hoodToPosition(0);
            }
        );
    }

    /**
     * Sets the hood to rotate to position of 3 rotations (24.1 degrees)
     * from the hoods starting position when the robot was turned on.
     * 
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to move hood to position 3
     */
    public Command hoodToPosition3(){
        return this.runOnce(
            () -> {
                this.hoodIO.hoodToPosition(3);
            }
        );
    }

    /**
     * Sets the hood to rotate to position of 5 rotations (40.16 degrees)
     * from the hoods starting position when the robot was turned on.
     * 
     * <b>NOTE: Start the robot with the hood in the fully retracted position!</b>
     * @return Command to move hood to position 5
     */
    public Command hoodToPosition5(){
        return this.runOnce(
            () -> {
                this.hoodIO.hoodToPosition(5);
            }
        );
    }
    
    public boolean checkSetpoint() {
        double closedLoopErrorDegrees = hoodIO.getRotations().getDegrees() - setPointHoodDegree;
        this.hoodIO.getRotations();
        boolean atSetPoint = Math.abs(closedLoopErrorDegrees) < 1;
        Logger.recordOutput("HoodSubsystem/Setpoints/atSetPoint", setPointHoodDegree);
        Logger.recordOutput("HoodSubsystem/Setpoints/closedLoopError", closedLoopErrorDegrees);
        return atSetPoint;
    }

    @Override
    public void periodic() {
        this.hoodIO.updateInputs(inputs);
        Logger.processInputs("HoodSubsystem", inputs);
    }

}

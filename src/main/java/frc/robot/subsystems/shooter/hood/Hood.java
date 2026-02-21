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

    public Command hoodToPosition0(){
        return this.runOnce(
            () -> {
                this.hoodIO.hoodToPosition(0);
            }
        );
    }

    public Command hoodToPosition3(){
        return this.runOnce(
            () -> {
                this.hoodIO.hoodToPosition(3);
            }
        );
    }

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

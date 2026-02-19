package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

public class Hood implements Subsystem {
    private double power = 0;
    double setPointHoodDegree;
    HoodIO hoodIO;
    HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood() {
        this.hoodIO = new HoodIOTalonFX();
    }

    public void hoodToAngle(double angle){
        hoodIO.hoodToAngle(angle);
        this.setPointHoodDegree = angle;
        Logger.recordOutput("HoodSubsystem/Setpoints/setpoint", angle);
    }

    public boolean checkSetpoint(){
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

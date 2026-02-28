package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    double angle = 0;
    double power = 0;
    IntakeIO intakeIO;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.intakeIO = io;
    }

    public Command intakeRunRollers() {
        return this.runOnce(
                () -> {
                    // Set power to full (1)
                    this.power = 1;

                    // Use power to start the IntakeIO Hardware motor
                    this.intakeIO.rollersRunVolts(this.power);
                }).unless(
                    () -> {
                        return (this.intakeIO.getRotations().getRotations() < 8.0);
                    }
                );
    }

    public Command intakeStopRollers() {
        return this.runOnce(
                () -> {
                    this.intakeIO.rollersStop();
                });
    }

    /**
     * For testing purposes of incrementing the position of the intake up
     * 
     * @return Command to increment or increase the intakes pivot position.
     */
    public Command intakePivotIncrements() {
        return this.runOnce(
                () -> {
                    this.angle += 1;
                    this.angle = MathUtil.clamp(this.angle, 0.0, 45.0);
                    this.intakeIO.intakePivotToAngle(this.angle);
                });
    }

    /**
     * For testing purposes of decrementing the position of the intake down.
     * 
     * @return Command to decrement or decrease the intakes pivot position.
     */
    public Command intakePivotDecrements() {
        return this.runOnce(
                () -> {
                    this.angle -= 1;
                    this.angle = MathUtil.clamp(this.angle, 0.0, 45.0);
                    this.intakeIO.intakePivotToAngle(this.angle);
                });
    }

    /**
     * Sets intake pivot motor to a certain angle based on what we put in to the
     * double
     * 
     * @param angle Angle in degrees
     * @return Command to set the Intake Pivot Motor at angle for shooting
     */
    public Command intakePivotToAngle(double angle) {
        return this.runOnce(
                () -> {
                    this.angle = angle;
                    this.intakeIO.intakePivotToAngle(this.angle);
                });
    }

    /**
     * Retracts the Intake Pivot Motor to its starting position
     * <p>
     * <b>NOTE: Start the robot with the Intake Pivot Motor in the fully retracted
     * position!</b>
     * 
     * @return Command to set Intake Pivot Motor to starting position
     */
    public Command intakePivotAngleRetract() {
        this.angle = Constants.IntakeConstants.intakePivotRetractAngle;
        return this.intakePivotToAngle(this.angle);
    }

    /**
     * Sets intake pivot motor to a certain angle based on what we put in to the
     * double
     * <p>
     * <b>NOTE: Start the robot with the Intake Pivot Motor in the fully retracted
     * position!</b>
     * 
     * @return Command to set Intake Pivot Motor for close shooting
     */
    public Command intakePivotAngleExtend() {
        return this.intakePivotToAngle(Constants.IntakeConstants.intakePivotOutAngle);
    }

    /**
     * Returns whether the Intake Pivot Motor is at its set point distance, given a
     * percent of tolerence
     * specified in the IntakeIO hardware class
     * 
     * @return BooleanSupplier: True Intake Pivot Motor is at its setpoint, false
     *         otherwise
     */
    public BooleanSupplier intakePivotAtPositionSetpoint() {
        return () -> this.intakeIO.intakeAtPositionSetpoint();
    }

    public Command deployIntake(){
        return Commands.sequence(
            this.intakePivotAngleExtend(), this.intakeRunRollers()
        );
    }

    public Command retractIntake(){
        return Commands.parallel(
            this.intakePivotAngleRetract(), this.intakeStopRollers()
        );
    }

    public Command toggleIntake(){
        if (this.intakeIO.getRotations().getRotations() <7.0){
            // this.toggleIntake() = true;
            return this.deployIntake();

        } else {
            // this.toggleIntake = false;
            return this.retractIntake();
            
        }
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update inputs object with the current status of the KickerIO hardware
        // and then write values to the Log
        this.intakeIO.updateInputs(inputs);
        Logger.processInputs("IntakeSubsystem", inputs);
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}

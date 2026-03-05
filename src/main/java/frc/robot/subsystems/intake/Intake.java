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
    boolean intakeIsExtended = false;
    IntakeIO intakeIO;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.intakeIO = io;

        // On intake initialization, find out if intake is extended or retracted
        intakeIsExtended = this.intakeIO.getRotations().getRotations() > 0.25 ? true : false;
    }

    public Command intakeResetCanCoder() {
        return this.runOnce(
            () -> this.intakeIO.intakeResetCanCoder()
        );
    }

    // Sets pivot to brake on true, coast on false
    public Command setBrakeMode(boolean brakeMode) {
        return this.runOnce(() -> this.intakeIO.setBrakeMode(brakeMode));
    }

    public Command intakeRunRollers() {
        return this.run(
                () -> {
                    // Set power to full (1)
                    this.power = 0.67;

                    // Use power to start the IntakeIO Hardware motor
                    this.intakeIO.rollersRunVolts(this.power);
                }
                // ).unless(
                //     () -> {
                //         return (this.intakeIO.getRotations().getRotations() < 0.2);
                //     }
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
                    this.angle = MathUtil.clamp(this.angle, 0.0, 0.385);
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
                    this.angle = MathUtil.clamp(this.angle, 0.0, 0.385);
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
       Logger.recordOutput("Pivot/Angle", angle);
        return this.run(
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
        this.angle = Constants.IntakeConstants.intakePivotRetractAngleDegrees;
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
        return this.intakePivotToAngle(Constants.IntakeConstants.intakePivotOutAngleDegrees);
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
        Logger.recordOutput("Pivot/Angle", angle);
        return () -> this.intakeIO.intakeAtPositionSetpoint();
    }

    public Command deployIntake(){
        return Commands.sequence(
            this.intakePivotAngleExtend().until(this.intakePivotAtPositionSetpoint()), this.intakeRunRollers().until(() -> true)
        );
    }

    public Command retractIntake(){
        return Commands.sequence(
         this.intakeStopRollers().until(() -> true), this.intakePivotAngleRetract().until(this.intakePivotAtPositionSetpoint())
        );
    }

    // Toggle intake in and out, does not start rollers. Rollers are controlled
    // by both controllers LTs
    public Command toggleIntake(){
       
        if (this.intakeIsExtended){
            // INTAKE IS EXTENDED
            // 1. Toggle the intake state
            this.intakeIsExtended = false;

            // 2. Retract and set brake mode
            return Commands.sequence(
                this.intakePivotAngleRetract().until(this.intakePivotAtPositionSetpoint()),
                this.setBrakeMode(true)
            );

        } else {
            // INTAKE IS RETRACTED
            // 1. Toggle the intake state
            this.intakeIsExtended = true;

            // 2. Extend and set coast mode
            return Commands.sequence(
                this.intakePivotAngleExtend().until(this.intakePivotAtPositionSetpoint()),
                this.setBrakeMode(false)
            );
            
        }
    }

    public Boolean isExtended(){
        return this.intakeIO.getRotations().getRotations() > 0.25;
    }

        public Boolean isNotExtended(){
        return this.intakeIO.getRotations().getRotations() <= 0.25;
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

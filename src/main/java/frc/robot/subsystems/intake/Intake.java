package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  // Reset the intake cancoder offset when pivot is at upper hard stop.
  // Used for testing.
  public Command intakeResetCanCoder() {
    return this.runOnce(
        () -> this.intakeIO.intakeResetCanCoder()).withName("intakeResetCanCoder()");
  }

  // Sets pivot to brake on true, coast on false
  public Command setBrakeMode(boolean brakeMode) {
    return this.runOnce(() -> this.intakeIO.setBrakeMode(brakeMode)).withName("setBrakeMode()");
  }

  public Command intakeRunRollers() {
    return this.run(
        () -> {
          // Set roller power
          this.power = Constants.IntakeConstants.intakeRollerPower;

          // Use power to start the IntakeIO Hardware motor
          this.intakeIO.rollersRunVolts(this.power);
        }).withName("intakeRunRollers()");
  }

  public Command intakeStopRollers() {
    return this.runOnce(
        () -> {
          this.intakeIO.rollersStop();
        }).withName("intakeStopRollers()");
  }

  /**
   * Sets intake pivot motor to a certain angle based on what we put in to the
   * double
   * 
   * @param angle
   *          Angle in degrees
   * @return Command to set the Intake Pivot Motor at angle for shooting
   */
  public Command intakePivotToAngle(double angle) {
    return this.run(
        () -> {
          this.angle = angle;
          Logger.recordOutput("Pivot/Angle", this.angle);
          this.intakeIO.intakePivotToAngle(this.angle);
        }).withName("intakePivotToAngle()");

  }

  /**
   * Retracts the Intake Pivot Motor to its starting position
   * <p>
   * <b>NOTE: Start the robot with the Intake Pivot Motor in the fully retracted
   * position!</b>
   * 
   * @return Command to set Intake Pivot Motor to starting position
   */
  public Command retractIntake() {
    return this.intakePivotToAngle(Constants.IntakeConstants.intakePivotRetractAngleDegrees)
        .withName("retractIntake()");
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
  public Command deployIntake() {
    return this.intakePivotToAngle(Constants.IntakeConstants.intakePivotDeployAngleDegrees).withName("deployIntake()");
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

  /**
   * Toggle the Intake Pivot out and in. This uses the getRotations() of the
   * subsystem
   * to determine if it is currently retracted or extended.
   * 
   * @return Command to either extend or retract the intake subsystem
   */
  public Command toggleIntakePivot() {
    return this.runOnce(
        () -> {
          // Since this command needs to make a decision, we must handle
          // the decision making inside of the lambda!
          if (this.intakeIO.getRotations().getRotations() < 0.4) {
            this.angle = Constants.IntakeConstants.intakePivotDeployAngleDegrees;
            this.intakeIO.intakePivotToAngle(this.angle);
            this.intakeIO.setBrakeMode(false);
          } else {
            this.angle = Constants.IntakeConstants.intakePivotRetractAngleDegrees;
            this.intakeIO.intakePivotToAngle(this.angle);
            this.intakeIO.setBrakeMode(true);
          }
        }).withName("toggleIntakePivot()");
  }

  public double getIntakeAngleSetpoint() {
    return this.angle;
  }

  public Command intakePulsePivot() {
    return Commands.repeatingSequence(
            Commands.race(
                this.intakePivotToAngle(Constants.IntakeConstants.intakePivotPulseUpAngleDegrees),
                new WaitCommand(1)),
            Commands.race(
                this.intakePivotToAngle(Constants.IntakeConstants.intakePivotDeployAngleDegrees),
                new WaitCommand(1)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update inputs object with the current status of the KickerIO hardware
    // and then write values to the Log
    this.intakeIO.updateInputs(inputs);
    Logger.processInputs("IntakeSubsystem", inputs);
    Logger.recordOutput("IntakeSubsystem/DefaultCommand",
        this.getDefaultCommand() != null ? this.getDefaultCommand().getName() : "N/A");
    Logger.recordOutput("IntakeSubsystem/CurrentCommand", 
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "N/A");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

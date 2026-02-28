package frc.robot.subsystems.intake;

public interface IntakeIO {

    public void updateInputs(IntakeIOInputs inputs);
 
        /**
     * Sets the intake pivot to the position (number of rotations from start) based on
     * provided angle.
     * @param angle Angle in degrees. Valid from 0 to 45 degrees max
     */
    public void intakePivotToAngle(double angle);

    /**
     * Sets the intake pivot to move to a certian positon, otherwise known as
     * number of rotations from the hood's starting position when the
     * robot was first turned on.
     * @param position Number of rotations
     */
    public void intakePivotToPosition(double position);

    /**
     * Gets the difference between the motor's position (number of rotations) from its current setpoint
     * @return Error amount
     */
    public double getIntakePivotPositionError();

    /**
     * Returns whether the hood is at its set point distance, given a percent of tolerence.
     * @return True if hood is at setpoint, false otherwise
     */
    public boolean intakeAtPositionSetpoint();

    public void rollersRunVolts(double power);

    public void rollersStop();
    
}

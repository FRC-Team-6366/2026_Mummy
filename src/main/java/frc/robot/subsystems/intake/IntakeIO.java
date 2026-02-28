package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    /**
     * Gets the current number of rotations of the intake pivot motor from its
     * starting
     * position
     * <p>
     * <b>NOTE: Move the hood to fully restracted position before turning on the
     * robot!</b>
     * 
     * @return Rotation2d object representing the number of rotations of the intake
     *         pivot motor
     */
    public Rotation2d getRotations();

    /**
     * Sets the intake pivot motor to the position (number of rotations from start)
     * based on
     * provided angle.
     * 
     * @param angle Angle in degrees. Valid from 0 to 45 degrees max
     */
    public void intakePivotToAngle(double angle);

    /**
     * Sets the intake pivot motor to move to a certian positon, otherwise known as
     * number of rotations from the intake pivot motor's starting position when the
     * robot was first turned on.
     * 
     * @param position Number of rotations
     */
    public void intakePivotToPosition(double position);

    /**
     * Gets the difference between the intake pivot motor's position (number of
     * rotations) from its current setpoint
     * 
     * @return Error amount
     */
    public double getIntakePivotPositionError();

    /**
     * Returns whether the hood is at its set point distance, given a percent of
     * tolerence.
     * 
     * @return True if hood is at setpoint, false otherwise
     */
    public boolean intakeAtPositionSetpoint();

    public void rollersRunVolts(double power);

    public void rollersStop();

    public void updateInputs(IntakeIOInputs inputs);

}

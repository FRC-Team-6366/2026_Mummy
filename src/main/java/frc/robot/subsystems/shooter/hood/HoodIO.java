package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    /**
     * Gets the current number of rotations of the hood motor from its starting
     * position
     * <p>
     * <b>NOTE: Move the hood to fully restracted position before turning on the robot!</b>
     * @return Rotation2d object representing the number of rotations of the hood motor
     */
    public Rotation2d getRotations();

    /**
     * Sets the hood to the position (number of rotations from start) based on
     * provided angle.
     * @param angle Angle in degrees. Valid from 0 to 45 degrees max
     */
    public void hoodToAngle(double angle);

    /**
     * Sets the hood to move to a certian positon, otherwise known as
     * number of rotations from the hood's starting position when the
     * robot was first turned on.
     * @param position Number of rotations
     */
    public void hoodToPosition(double position);

    /**
     * Gets the difference between the motor's position (number of rotations) from its current setpoint
     * @return Error amount
     */
    public double getHoodPositionError();

    /**
     * Returns whether the hood is at its set point distance, given a percent of tolerence.
     * @return True if hood is at setpoint, false otherwise
     */
    public boolean hoodAtPositionSetpoint();

    /**
     * Updates the supplied inputs object with the current status of the 
     * hood motor.
     * @param inputs
     */
    public void updateInputs(HoodIOInputs inputs);
}

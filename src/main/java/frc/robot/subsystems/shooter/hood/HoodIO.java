package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    public Rotation2d getRotations();

    public void hoodToAngle(double angle);

    /**
     * Sets the hood to move to a certian positon, otherwise known as
     * number of rotations from the hood's starting position when the
     * robot was first turned on
     * @param position Number of rotations
     */
    public void hoodToPosition(double position);

    /**
     * Updates the supplied inputs object with the current status of the 
     * hood motor
     * @param inputs
     */
    public void updateInputs(HoodIOInputs inputs);
}

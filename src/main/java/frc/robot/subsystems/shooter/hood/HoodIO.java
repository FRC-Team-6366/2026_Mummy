package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {
    public Rotation2d getRotations();

    public void hoodToAngle(double angle);

    public void hoodToPosition(double position);

    public void updateInputs(HoodIOInputs inputs);
}

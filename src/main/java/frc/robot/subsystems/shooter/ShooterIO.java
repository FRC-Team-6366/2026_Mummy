package frc.robot.subsystems.shooter;

public interface ShooterIO {
    
    /**
     * Sets the shooters velocity using rotations per second for shooting fuel.
     * 
     * @param rps Rotations per second
     */
    public void setShooterVelocityRPS(double rps);

    /**
     * Sets the shooters velocity using feet per second for shooting fuel.
     * 
     * @param feetPerSecond feet per second (ft/s)
     */
    public void setShooterVelocityFeetPerSecond(double feetPerSecond);

    /**
     * Returns the amount of error between the shooter's current velocity
     * and the shooter's setpoint velocity;
     * @return Difference of rotations per second
     */
    public double getShooterVelocityError();

    /**
     * Checks if the shooter is at the requested velocity setpoint
     * @return True if shooter is at the velocity setpoint. False otherwise
     */
    public boolean shooterAtVelocitySetPoint();

    /**
     * Updates the supplied inputs objects with the current status of the 
     * shooter motor
     * @param inputs
     */
    public void updateInputs(ShooterIOInputs inputs);
    
}

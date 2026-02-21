package frc.robot.subsystems.shooter;

public interface ShooterIO {

    /**
     * Sets the power of the Shooter. Will used the supplied value and 
     * convert it to a value between -12.0 volts and 12.0 volts
     * @param power Power value between -1.0 and 1.0
     */
    public void setShooterPower(double power);
    
    
    public void setShooterVelocity(double rps);

    /**
     * Updates the supplied inputs objects with the current status of the 
     * shooter motor
     * @param inputs
     */
    public void updateInputs(ShooterIOInputs inputs);
    
}

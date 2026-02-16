package frc.robot.subsystems.kicker;

public interface KickerIO {
    /**
     * Sets the power of the kicker. Will used the supplied value and 
     * convert it to a value between -12.0 volts and 12.0 volts
     * @param power Power value between -1.0 and 1.0
     */
    public void setKickPower(double power);
    
    /**
     * Updates the supplied inputs objects with the current status of the 
     * kicker motor
     * @param inputs
     */
    public void updateInputs(KickerIOInputs inputs);
}

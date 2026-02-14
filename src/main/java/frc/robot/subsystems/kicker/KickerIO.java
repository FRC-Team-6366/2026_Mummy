package frc.robot.subsystems.kicker;

public interface KickerIO {
    public void setKickPower(double power);
    
    public void updateInputs(KickerIOInputs inputs);
}

package frc.robot.subsystems.intake;

public interface IntakeIO {

    public void updateInputs(IntakeIOInputs inputs);
 
    public void rollersRunVolts(double power);

    public void rollersStop();
    
}

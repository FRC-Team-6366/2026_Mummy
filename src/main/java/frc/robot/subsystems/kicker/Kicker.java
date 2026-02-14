package frc.robot.subsystems.kicker;



import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
    private final TalonFX kickMotor;
    double power = 0;
    KickerIO kickerIO;
    KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
    public Kicker(){
        kickMotor = new TalonFX(24);
        TalonFXConfiguration kickConfiguration = new TalonFXConfiguration();
        kickConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        kickMotor.getConfigurator().apply(kickConfiguration);
    }
    public void setPower(double power){
        double voltage = power *12;
        VoltageOut volts = new VoltageOut(voltage);
        kickMotor.setControl(volts);
    }

    public void stop(){
        // double voltage = 0;
        // VoltageOut volts = new VoltageOut(voltage);
        // kickMotor.setControl(volts);
        if (this.power>0){
            this.power =0;
        }
        this.setPower(power);
    }

    public void kickIncrements(){
        if (this.power<1){
            this.power +=0.3;

        }
        this.setPower(power);
    }
    public void kickDecrements(){
        if (this.power>0){
            this.power -=0.3;

        }
        this.setPower(power);
    }



     @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //2.14.2026 I think this is an auto generated comment above. no clue what it means
    this.kickerIO.updateInputs(inputs);
    Logger.processInputs("IndexerSubstem", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
    
}

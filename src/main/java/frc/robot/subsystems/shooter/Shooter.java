package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final TalonFX motor1;
    private final TalonFX motor2;

    double power = 0;

    public Shooter(int motorId1, int motorId2){
        this.motor1 = new TalonFX(motorId1);
        this.motor2 = new TalonFX(motorId2);

        TalonFXConfiguration fx = new TalonFXConfiguration();

        fx.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        this.motor2.getConfigurator().apply(fx);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        this.motor1.getConfigurator().apply(cfg);

    }

    public void setPower(double power){
    double voltage = 12 * power;
    VoltageOut volts = new VoltageOut(voltage);
    motor1.setControl(volts);
    motor2.setControl(volts);
    }



      public void incrementShooterVoltage() {
    if (this.power < 1) {
      this.power += 0.3;
      this.setPower(power);
    }
  }

  public void decrementVoltage() {
    if (this.power > 0) {
      this.power -= 0.3;
      this.setPower(power);
    }
  }

      public void stop(){
        if (this.power>0){
            this.power =0;
        }
        this.setPower(power);
    }
    

      @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

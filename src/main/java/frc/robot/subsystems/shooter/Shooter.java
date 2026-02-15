package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private double power = 0;
    ShooterIO shooterIO;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(){
        this.shooterIO = new ShooterIOTalonFX();
    }


    
    public void stop(){
        if (this.power>0){
            this.power =0;
        }
        this.shooterIO.setShooterPower(power);
    }

    public void shooterIncrements(){
        if (this.power<1){
            this.power +=0.3; //0.2

        }
        this.shooterIO.setShooterPower(power);
    }
    public void shooterDecrements(){
        if (this.power>0){
            this.power -=0.3;//0.2

        }
        this.shooterIO.setShooterPower(power);
    }



     @Override
  public void periodic() {
    this.shooterIO.updateInputs(inputs);
    Logger.processInputs("IndexerSubsystem", inputs);
  }
}

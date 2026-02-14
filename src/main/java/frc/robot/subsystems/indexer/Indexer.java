package frc.robot.subsystems.indexer;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private double power = 0;
    IndexerIO indexerIO;
    IndexerIOInputs inputs = new IndexerIOInputsAutoLogged();

    public Indexer(){
        this.indexerIO = new IndexerIOTalonFX();
    }


    
    public void stop(){
        // double voltage = 0;
        // VoltageOut volts = new VoltageOut(voltage);
        // kickMotor.setControl(volts);
        if (this.power>0){
            this.power =0;
        }
        this.indexerIO.setIndexerPower(power);
    }

    public void indexIncrements(){
        if (this.power<1){
            this.power +=0.05; //0.2

        }
        this.indexerIO.setIndexerPower(power);
    }
    public void indexDecrements(){
        if (this.power>0){
            this.power -=0.05;//0.2

        }
        this.indexerIO.setIndexerPower(power);
    }



     @Override
  public void periodic() {
    this.indexerIO.updateInputs(inputs);
    Logger.processInputs("IndexerSubsystem", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
    

}

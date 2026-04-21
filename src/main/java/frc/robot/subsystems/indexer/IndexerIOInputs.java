package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IndexerIOInputs {
  boolean connected = false;
  double indexVolts = 0;
  double indexRps = 0;
  double indexCurrent = 0;
  double indexSupplyCurrent = 0;
}

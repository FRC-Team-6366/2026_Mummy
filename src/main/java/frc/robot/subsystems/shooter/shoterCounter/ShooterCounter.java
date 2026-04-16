package frc.robot.subsystems.shooter.shoterCounter;

public class ShooterCounter {
  private int fuelCounter = 0;
  private double timeRecordPeriod = 0.5;
  private long resetTime;
  private double avgBallsPerTimePeriod;

  public ShooterCounter() {
    this.resetTime();
  }
  
  public ShooterCounter(double timeRecordPeriod) {
    this.timeRecordPeriod = timeRecordPeriod;
    this.avgBallsPerTimePeriod = 0;
    this.resetTime();
  }

  public synchronized void incrementShot() {
    this.fuelCounter++;
  }

  public int getFuelCounter() {
    return this.fuelCounter;
  }

  public synchronized void setTimeRecordPeriod(double timeRecordPeriod) {
    this.timeRecordPeriod = timeRecordPeriod;
  }

  public double getAvgBallsPerTimePeriod() {
    return avgBallsPerTimePeriod;
  }

  public double getAvgBallsPerSecond() {
    double timeRatio = 1 / this.timeRecordPeriod;
    return avgBallsPerTimePeriod * timeRatio;
  }

  public synchronized void updateAvgFuelPerTimePeriod() {
    this.avgBallsPerTimePeriod = (double)this.fuelCounter / this.timeRecordPeriod;
  }

  public synchronized void resetTime() {
    this.resetTime = System.currentTimeMillis() + (long)this.timeRecordPeriod * 1000;
  }

  public void reset() {
    this.fuelCounter = 0;
    this.avgBallsPerTimePeriod = 0;
    this.resetTime();
  }

  public void periodic() {
    long currentTime = System.currentTimeMillis();
    if (currentTime > resetTime) {
      this.updateAvgFuelPerTimePeriod();
      this.fuelCounter = 0;
      this.resetTime();
    }
  }
}

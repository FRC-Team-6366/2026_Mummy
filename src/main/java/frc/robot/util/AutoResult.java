package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * ENUM for tracking whether the won or lost the autonomous period
 */
public enum AutoResult {
  WON,LOST,UNDEFINED;

  /**
   * Uses the team's Alliance color as well as the game specific message to determine
   * if the team won or lost autonomous. If no game specific message is found, then 
   * this will return "UNDEFINED"
   * @return <b>AutoResult value</b>: AutoResult.WON, AutoResult.Lost, AutoResult.UNDEFINED
   * @see DriverStation#getAlliance()
   * @see DriverStation#getGameSpecificMessage()
   * @see DriverStation.Alliance
   */
  public static AutoResult getAutoResult() {
    // Only check if the robot is in teleop
    if (DriverStation.isTeleopEnabled()) {
      // Handle case when the team is Blue Alliance
      if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        
        // Handle case when Blue Alliance wins Auto period (We Won! 😄)
        if (DriverStation.getGameSpecificMessage().contains("B")) {
          return AutoResult.WON;
        } 
        
        // Handle case when Red Alliance wins Auto period (We Lost 😭)
        else if (DriverStation.getGameSpecificMessage().contains("R")) {
          return AutoResult.LOST;
        } 
        
        // Handle case where no message is sent to the Driver Station (Who knows? 😕)
        else {
          return AutoResult.UNDEFINED;
        }
      } 
      
      // Handle case when the team is Red Alliance
      else {
        
        // Handle case when Red Alliance wins Auto period (We Won! 😄)
        if (DriverStation.getGameSpecificMessage().contains("R")) {
          return AutoResult.WON;
        } 
        
        // Handle case when Blue Alliance wins Auto period (We Lost 😭)
        else if (DriverStation.getGameSpecificMessage().contains("B")) {
          return AutoResult.LOST;
        } 
        
        // Handle case where no message is sent to the Driver Station (Who knows? 😕)
        else {
          return AutoResult.UNDEFINED;
        }
      }
  } else {
    return AutoResult.UNDEFINED;
  }
  }

  public String getAsColorString() {
    if (this == AutoResult.WON)
      return "#00FF00";
    else if (this == AutoResult.LOST)
      return "#FF0000";
    else
      return "#000000";
  }
}

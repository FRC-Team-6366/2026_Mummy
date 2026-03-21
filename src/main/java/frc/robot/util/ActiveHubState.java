package frc.robot.util;

public enum ActiveHubState {
  ACTIVE, INACTIVE, TRANSITIONING, UNDEFINED;

  public String getColor(){
    if (this == ACTIVE) {
      return "#00ff00";
    } else if (this == INACTIVE){
      return "#ff0000";
    } else if (this == TRANSITIONING){
      return "#ffff00";
    } else {
      return "#000000";
    }
  }

}

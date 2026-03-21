package frc.robot.util;

public  enum GameTimeMarkers {
  SHIFT_TRANSITION_RUNNING,
  SHIFT_1_WARNING,
  SHIFT_1_RUNNING,
  SHIFT_2_WARNING,
  SHIFT_2_RUNNING,
  SHIFT_3_WARNING,
  SHIFT_3_RUNNING,
  SHIFT_4_WARNING,
  SHIFT_4_RUNNING,
  SHIFT_END_WARNING,
  SHIFT_END_RUNNING,
  UNDEFINED;

  static int warningTime =5;

  public int getTime(){
    switch (this){
      case SHIFT_1_RUNNING:
        return 130;
      case SHIFT_1_WARNING:
        return 130 +warningTime;
      case SHIFT_2_RUNNING:
        return 105;
      case SHIFT_2_WARNING:
        return 105 + warningTime;
      case SHIFT_3_RUNNING:
        return 80;
      case SHIFT_3_WARNING:
        return 80+ warningTime;
      case SHIFT_4_RUNNING:
        return 55;
      case SHIFT_4_WARNING:
        return 55 + warningTime;
      case SHIFT_END_RUNNING:
        return 30;
      case SHIFT_END_WARNING:
        return 30+ warningTime;
      case SHIFT_TRANSITION_RUNNING:
        return 140;
      case UNDEFINED:
        return -1;
// -----------------------        
      default:
        return -1;

    }
  }

  public static int getTime(GameTimeMarkers marker){
        switch (marker){
      case SHIFT_1_RUNNING:
        return 130;
      case SHIFT_1_WARNING:
        return 130 +warningTime;
      case SHIFT_2_RUNNING:
        return 105;
      case SHIFT_2_WARNING:
        return 105 + warningTime;
      case SHIFT_3_RUNNING:
        return 80;
      case SHIFT_3_WARNING:
        return 80+ warningTime;
      case SHIFT_4_RUNNING:
        return 55;
      case SHIFT_4_WARNING:
        return 55 + warningTime;
      case SHIFT_END_RUNNING:
        return 30;
      case SHIFT_END_WARNING:
        return 30+ warningTime;
      case SHIFT_TRANSITION_RUNNING:
        return 140;
      case UNDEFINED:
        return -1;
// -----------------------        
      default:
        return -1;

    }

  }

}
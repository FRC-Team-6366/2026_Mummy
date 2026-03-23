package frc.robot.util;

import java.util.LinkedList;

/**
 * ENUM for tracking the different time periods of the teleoperator
 * period of the 2026 FRC competition
 * @author Hayden
 * @author Will E
 * @since 2026
 */
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

  static int warningTimeOffset = 5;

  static LinkedList<GameTimeMarkers> oddShifts = new LinkedList<GameTimeMarkers>();
  static LinkedList<GameTimeMarkers> evenShifts = new LinkedList<GameTimeMarkers>();
  static LinkedList<GameTimeMarkers> warningShifts = new LinkedList<GameTimeMarkers>();
  static LinkedList<GameTimeMarkers> warningShifts_1_3_End = new LinkedList<GameTimeMarkers>();
  static LinkedList<GameTimeMarkers> warningShifts_2_4_End = new LinkedList<GameTimeMarkers>();

  public int getTime(){
    switch (this){
      case SHIFT_1_RUNNING:
        return 130;
      case SHIFT_1_WARNING:
        return 130 + warningTimeOffset;
      case SHIFT_2_RUNNING:
        return 105;
      case SHIFT_2_WARNING:
        return 105 + warningTimeOffset;
      case SHIFT_3_RUNNING:
        return 80;
      case SHIFT_3_WARNING:
        return 80 + warningTimeOffset;
      case SHIFT_4_RUNNING:
        return 55;
      case SHIFT_4_WARNING:
        return 55 + warningTimeOffset;
      case SHIFT_END_RUNNING:
        return 30;
      case SHIFT_END_WARNING:
        return 30 + warningTimeOffset;
      case SHIFT_TRANSITION_RUNNING:
        return 140;
      case UNDEFINED:
        return -1;
// -----------------------        
      default:
        return -1;

    }
  }

  /**
   * Returns the start time for a specific shift or shift transision period
   * <p>
   * <b>Periods - Times</b>
   * <ul>
   * <li><b>SHIFT_TRANSITION_RUNNING</b> - 140</li>
   * <li><b>SHIFT_1_WARNING</b> - 135</li>
   * <li><b>SHIFT_1_RUNNING</b> - 130</li>
   * <li><b>SHIFT_2_WARNING</b> - 110</li>
   * <li><b>SHIFT_2_RUNNING</b> - 105</li>
   * <li><b>SHIFT_3_WARNING</b> - 85</li>
   * <li><b>SHIFT_3_RUNNING</b> - 80</li>
   * <li><b>SHIFT_4_WARNING</b> - 60</li>
   * <li><b>SHIFT_4_RUNNING</b> - 55</li>
   * <li><b>SHIFT_END_WARNING</b> - 35</li>
   * <li><b>SHIFT_END_RUNNING</b> - 30</li>
   * </ul>
   * </p>
   * @param marker GameTimeMarkers ENUM value
   * @return Seconds from the end of the match
   */
  public static int getTime(GameTimeMarkers marker){
        switch (marker){
      case SHIFT_1_RUNNING:
        return 130;
      case SHIFT_1_WARNING:
        return 130 +warningTimeOffset;
      case SHIFT_2_RUNNING:
        return 105;
      case SHIFT_2_WARNING:
        return 105 + warningTimeOffset;
      case SHIFT_3_RUNNING:
        return 80;
      case SHIFT_3_WARNING:
        return 80+ warningTimeOffset;
      case SHIFT_4_RUNNING:
        return 55;
      case SHIFT_4_WARNING:
        return 55 + warningTimeOffset;
      case SHIFT_END_RUNNING:
        return 30;
      case SHIFT_END_WARNING:
        return 30+ warningTimeOffset;
      case SHIFT_TRANSITION_RUNNING:
        return 140;
      case UNDEFINED:
        return -1;
// -----------------------        
      default:
        return -1;

    }

  }

  /**
   * Returns list of GameTimeMarkers Enums for both odd shifts as well as
   * the tranisitin runnning and end running shifts
   * @return LinkedList of GameTimeMarkers ENUMs
   */
  public static LinkedList<GameTimeMarkers> getOddEnums(){
    if (oddShifts.size() == 0) {
      oddShifts.add(SHIFT_TRANSITION_RUNNING);
      oddShifts.add(SHIFT_1_RUNNING);
      oddShifts.add(SHIFT_3_RUNNING);
      oddShifts.add(SHIFT_END_RUNNING);
    }

    return oddShifts;
  }
  
  /**
   * Returns list of GameTimeMarkers Enums for both even shifts as well as
   * the tranisitin runnning and end running shifts
   * @return LinkedList of GameTimeMarkers ENUMs
   */
  public static LinkedList<GameTimeMarkers> getEvenEnums(){
    if (evenShifts.size() == 0) {
      evenShifts.add(SHIFT_TRANSITION_RUNNING);
      evenShifts.add(SHIFT_2_RUNNING);
      evenShifts.add(SHIFT_4_RUNNING);
      evenShifts.add(SHIFT_END_RUNNING);
    }

    return evenShifts;
  }

  /**
   * Returns list of GameTimeMarkers Enums for warning shifts between
   * the different running shifts
   * @return LinkedList of GameTimeMarkers ENUMs
   */
  public static LinkedList<GameTimeMarkers> getWarningEnums(){
    if (warningShifts.size() == 0) {
      warningShifts.add(SHIFT_1_WARNING);
      warningShifts.add(SHIFT_2_WARNING);
      warningShifts.add(SHIFT_3_WARNING);
      warningShifts.add(SHIFT_4_WARNING);
      warningShifts.add(SHIFT_END_WARNING);
    }

    return warningShifts;
  }

  /**
   * Returns list of GameTimeMarkers Enums for warning shifts 1, 3
   * and the ending shift
   * @return LinkedList of GameTimeMarkers ENUMs
   */
  public static LinkedList<GameTimeMarkers> getWarning_1_3_End_Enums(){
    if (warningShifts_1_3_End.size() == 0) {
      warningShifts_1_3_End.add(SHIFT_1_WARNING);
      warningShifts_1_3_End.add(SHIFT_3_WARNING);
      warningShifts_1_3_End.add(SHIFT_END_WARNING);
    }

    return warningShifts_1_3_End;
  }

  /**
   * Returns list of GameTimeMarkers Enums for warning shifts 2, 4, 
   * and the ending shift
   * @return LinkedList of GameTimeMarkers ENUMs
   */
  public static LinkedList<GameTimeMarkers> getWarning_2_4_End_Enums(){
    if (warningShifts_2_4_End.size() == 0) {
      warningShifts_2_4_End.add(SHIFT_2_WARNING);
      warningShifts_2_4_End.add(SHIFT_4_WARNING);
      warningShifts_2_4_End.add(SHIFT_END_WARNING);
    }

    return warningShifts_2_4_End;
  }

}
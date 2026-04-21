package frc.robot.util;

import java.util.LinkedList;

public enum ActiveHubState {
  /**
   * Marks that your hub is active and can be used for scoring. Color is Green
   */
  ACTIVE, 
  /**
   * Marks that your hub is not active and cannot be used for scoring. Color is Red
   */
  INACTIVE, 
  /**
   * Marks that the active hub is about to transition to active and should caution drivers on scoring. Color is Yellow/Green
   */
  TRANSITIONING_TO_ACTIVE,
  /**
   * Marks that the active hub is about to transition to inactive and should caution drivers on scoring. Color is Yellow/Red
   */
  TRANSITIONING_TO_INACTIVE, 
  /**
   * Marks that active hub status is unknown and possibly we don't know what time it is. Color is Black
   */
  UNDEFINED;

  /**
   * Returns the Hex color value representing the supplied ActiveHubState
   * @param state ActiveHubState value such as ACTIVE, INACTIVE, TRANSITIONING_TO_ACTIVE, TRANSITIONING_TO_INACTIVE
   * @return String Array of Hex Color String such as "#00ff00" where the format is "#RRGGBB"
   */
  public static String[] getMultiColor(ActiveHubState state){
    LinkedList<String> colors = new LinkedList<String>();
    if (state == ACTIVE) {
      colors.add("#00FF00");
      return colors.toArray(new String[1]);
    } 
    
    else if (state == INACTIVE){
      colors.add("#ff0000");
      return colors.toArray(new String[1]);
    } 
    
    else if (state == TRANSITIONING_TO_ACTIVE){
      for (int i = 0; i < 10; i++)
        colors.add("#ffff00");
      for (int i = 0; i < 10; i++)
        colors.add("#00FF00");
      return colors.toArray(new String[20]);
    } 

    else if (state == TRANSITIONING_TO_INACTIVE){
      for (int i = 0; i < 10; i++)
        colors.add("#ffff00");
      for (int i = 0; i < 10; i++)
        colors.add("#ff0000");
      return colors.toArray(new String[20]);
    } 
    
    else {
      colors.add("#000000");
      return colors.toArray(new String[1]);
    }
  }

  /**
   * Returns the Hex color value representing the supplied ActiveHubState
   * @param state ActiveHubState value such as ACTIVE, INACTIVE, TRANSITIONING
   * @return Hex Color String such as "#00ff00" where the format is "#RRGGBB"
   */
  public static String getColor(ActiveHubState state) {
    if (state == ACTIVE) {
      return "#00ff00";
    } else if (state == INACTIVE){
      return "#ff0000";
    } else if (state == TRANSITIONING_TO_ACTIVE || state == TRANSITIONING_TO_INACTIVE) {
      return "#ffff00";
    } else {
      return "#000000";
    }
  }

}

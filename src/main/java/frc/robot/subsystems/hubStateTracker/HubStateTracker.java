package frc.robot.subsystems.hubStateTracker;

import java.util.LinkedList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ActiveHubState;
import frc.robot.util.AutoResult;
import frc.robot.util.GameTimeMarkers;

public class HubStateTracker extends SubsystemBase{
  private GameTimeMarkers gameTimeMarker = GameTimeMarkers.UNDEFINED;
  private AutoResult autoResult = AutoResult.UNDEFINED;
  LinkedList<GameTimeMarkers> shitfsWeCareAbout;
  LinkedList<GameTimeMarkers> shitfsWeDontCareAbout;
  LinkedList<GameTimeMarkers> shiftsTransitioningToActive;
  LinkedList<GameTimeMarkers> shiftsTransitioningToInactive;
  private int gameMatchTime;

  private static HubStateTracker tracker;

  /**
   * Private constructor to support singleton design pattern to prevent
   * more than one instance of this class
   */
  private HubStateTracker() {}

  /**
   * Returns the singleton instance for HubStateTracker
   * @return HubStateTracker object
   */
  public static HubStateTracker getInstance() {
    if (HubStateTracker.tracker == null) {
      HubStateTracker.tracker = new HubStateTracker();
    }

    return HubStateTracker.tracker;
  }

  public Command runHubStateTracker() {
    return this.run(() -> {}).withName("runHubStateTracker");
  }

  /**
   * Resets the states of this tracker. Use in Robot.disableInit() to prepare
   * this tracker for the next telop run
   */
  public void reset() {
    // Reset Game Shift Period Trackers and related variables
    this.autoResult = AutoResult.UNDEFINED;
    this.gameTimeMarker = GameTimeMarkers.UNDEFINED;
    this.shitfsWeCareAbout = null;
    this.shitfsWeDontCareAbout = null;
    this.shiftsTransitioningToActive = null;
    this.shiftsTransitioningToInactive = null;
  }

  public GameTimeMarkers getCurrentShiftPeriod() {
    return this.gameTimeMarker;
  }

  /**
   * Updates the class variable "gameMatchTime" using DriverStation.getMatchTime() 
   * and uses it to determine what shift value the class variable "gameTimeMarker"
   * is set too. This should be called in robotPeriodic
   * @see #gameMatchTime
   * @see #gameTimeMarker
   * @see #robotPeriodic()
   */
  private void determineCurrentGameShiftPeriod() {
    // Convert Driver Station time to integer
    this.gameMatchTime = (int)DriverStation.getMatchTime();
    
    switch (gameTimeMarker){
      case UNDEFINED:
        if (DriverStation.isTeleopEnabled())
          this.gameTimeMarker=GameTimeMarkers.SHIFT_TRANSITION_RUNNING;
        break;
      
      case SHIFT_TRANSITION_RUNNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_1_WARNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_1_WARNING;
        }
        break;
      case SHIFT_1_WARNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_1_RUNNING)){
            this.gameTimeMarker=GameTimeMarkers.SHIFT_1_RUNNING;
          }
          break;
      
      case SHIFT_1_RUNNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_2_WARNING)){
            this.gameTimeMarker=GameTimeMarkers.SHIFT_2_WARNING;
          }
        break;
      
      case SHIFT_2_WARNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_2_RUNNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_2_RUNNING;
        }
        break;
      case SHIFT_2_RUNNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_3_WARNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_3_WARNING;
        }
        break;
      
      case SHIFT_3_WARNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_3_RUNNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_3_RUNNING;
        }
        break;

      case SHIFT_3_RUNNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_4_WARNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_4_WARNING;
        }
        break;
      
      case SHIFT_4_WARNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_4_RUNNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_4_RUNNING;
        }
        break;
      
      case SHIFT_4_RUNNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_END_WARNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_END_WARNING;
        }
        break;
      
      case SHIFT_END_WARNING:
        if (this.gameMatchTime == GameTimeMarkers.getTime(GameTimeMarkers.SHIFT_END_RUNNING)){
          this.gameTimeMarker=GameTimeMarkers.SHIFT_END_RUNNING;
        }
        break;
      
      //---------------
      // This handles the "SHIFT_END_RUNNING" case since it doesn't actually move to 
      // anything next as well as all other times
      default:
        break;

    }
  }

  /**
   * Used to set variables for what shift periods of the match the team's hub is active
   * and inactive.
   * @see #autoResult
   * @see #shitfsWeCareAbout
   * @see #shitfsWeDontCareAbout
   * @see #warningShifts
   */
  private void determineGameShiftPeriodsWeCareAbout() {
    // If we have not determined if we won or lost auto period
    if (this.autoResult == AutoResult.UNDEFINED) {
      this.autoResult = AutoResult.getAutoResult();

      // Set the periods we care based on if we won or lost auto
      // or do nothing if we have not determined who won yet
      if (this.autoResult == AutoResult.WON) {
        shitfsWeCareAbout = GameTimeMarkers.getEvenEnums();
        shitfsWeDontCareAbout = GameTimeMarkers.getOddEnums();
        shiftsTransitioningToActive = GameTimeMarkers.getWarning_2_4_End_Enums();
        shiftsTransitioningToInactive = GameTimeMarkers.getWarning_1_3_End_Enums();
      } else if (this.autoResult == AutoResult.LOST) {
        shitfsWeCareAbout = GameTimeMarkers.getOddEnums();
        shitfsWeDontCareAbout = GameTimeMarkers.getEvenEnums();
        shiftsTransitioningToActive = GameTimeMarkers.getWarning_1_3_End_Enums();
        shiftsTransitioningToInactive = GameTimeMarkers.getWarning_2_4_End_Enums();
      }

      // There is no "else" since we don't want to change away from
      // AutoResult.UNDEFINED if we haven't figured this out yet!
      
    }

    Logger.recordOutput("WonAuto", this.autoResult.getAsColorString());
  }

  /**
   * Using the dashboard Key "GoalActive", this sends to the dasboard the color of ACTIVE, TRANSITIONING 
   * or INACTIVE for displaying if the team's hub is active and should be fired upon. This method uses 
   * the class variable "gameTimeMarker" to decide if we are in a period of play were the team's goal is active and can score
   * @see #gameTimeMarker
   * @see #shitfsWeCareAbout
   * @see #shitfsWeDontCareAbout
   * @see #shiftsTransitioningToActive
   * @see #shiftsTransitioningToInactive
   * @see ActiveHubState
   */
  private void sendActiveGoalStateToDashboard() {
    // If we have initialized our shifts lists
    if (shitfsWeCareAbout != null && shitfsWeDontCareAbout != null) {
      // Check if we are in a period we care about
      if (shitfsWeCareAbout.contains(this.gameTimeMarker)) {
        Logger.recordOutput("GoalActive", ActiveHubState.getMultiColor(ActiveHubState.ACTIVE));
      } 
      
      // Check if we are in a period we don't care about
      else if (shitfsWeDontCareAbout.contains(this.gameTimeMarker)){
        Logger.recordOutput("GoalActive", ActiveHubState.getMultiColor(ActiveHubState.INACTIVE));
      }

      // Check if we are about to transition to active
      else if (shiftsTransitioningToActive.contains(this.gameTimeMarker)) {
        Logger.recordOutput("GoalActive", ActiveHubState.getMultiColor(ActiveHubState.TRANSITIONING_TO_ACTIVE));
      }

      // Check if we are about to transition to inactive
      else if (shiftsTransitioningToInactive.contains(this.gameTimeMarker)) {
        Logger.recordOutput("GoalActive", ActiveHubState.getMultiColor(ActiveHubState.TRANSITIONING_TO_INACTIVE));
      }
      
      // Else we are in an UNDEFINED state
      else {
        Logger.recordOutput("GoalActive", ActiveHubState.getMultiColor(ActiveHubState.UNDEFINED));
      }
    }

    else {
      Logger.recordOutput("GoalActive", ActiveHubState.getMultiColor(ActiveHubState.UNDEFINED));
    }
  }

  @Override
  public void periodic() {
    this.determineGameShiftPeriodsWeCareAbout();
    this.determineCurrentGameShiftPeriod();
    this.sendActiveGoalStateToDashboard();
  }


}

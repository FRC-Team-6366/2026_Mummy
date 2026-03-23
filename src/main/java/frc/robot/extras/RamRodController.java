package frc.robot.extras;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Custom Controller class based on Xbox Controller that adds quality of life features
 * <p><b>Features</b>
 * <ul>
 * <li>Precision for joystick deadband</li>
 * <li>Ramp for joystick input</li>
 * <li>Rumble for set time length</li>
 * </ul>
 * </p>
 * <p>Notes</p>
 * @author Will E
 * @since 2026
 */
public class RamRodController extends CommandXboxController {
  Long stopRumbleTime = null;
  Long coolOffRumbleTime = null;
  boolean ramp = false;
  double precision = 0.0;
  
  /**
   * Creates RamRod Controller using the supplied port
   * @param port Controller Port (0-3)
   */
  public RamRodController(int port) {
    super(port);
  }

  /**
   * Creates RamRod Controller using the supplied port. Allows the specifying if ramp
   * setting for joystick input should be used or not
   * @param port Controller Port (0-3)
   * @param ramp True means X-Y axises will use x^2 ramp, false is nomral operation
   */
  public RamRodController(int port, boolean ramp) {
    super(port);
    this.ramp = ramp;
  }

  /**
   * Creates RamRod Controller using the supplied port. Allows setting the deadband 
   * of X and Y axes for the joy sticks. Value is range of 0 to 1 (zero being no 
   * deadband and 1 being evenything is deadband and does nothing). Suggest starting 
   * at value of 0.15
   * @param port Controller Port (0-3)
   * @param percision Double value from 0.0 to 1.0
   */
  public RamRodController(int port, double precision) {
    super(port);
    this.setPrecision(precision);
  }

  /**
   * Creates RamRod Controller using the supplied port. Allows setting the deadband 
   * of X and Y axes for the joy sticks. Value is range of 0 to 1 (zero being no 
   * deadband and 1 being evenything is deadband and does nothing). Suggest starting 
   * at value of 0.15. Also Allows the specifying if ramp setting for joystick input 
   * should be used or not
   * @param port Controller Port (0-3)
   * @param percision Double value from 0.0 to 1.0
   * @param ramp True means X-Y axises will use x^2 ramp, false is nomral operation
   */
  public RamRodController(int port, double precision, boolean ramp) {
    super(port);
    this.setPrecision(precision);
    this.ramp = ramp;
  }

  @Override
  public double getLeftX() {
    double value = super.getLeftX();

    if(value < precision && value > -precision)
		{
			value = 0;
		}
    
    if(ramp)
		{
			if(value < 0)
			{
				value = -(Math.pow(value, 2));
			} else {
				value = Math.pow(value, 2);
			}
		}
		
		return value;
  }

  @Override
  public double getLeftY() {
    double value = super.getLeftY();

    if(value < precision && value > -precision)
		{
			value = 0;
		}
    
    if(ramp)
		{
			if(value < 0)
			{
				value = -(Math.pow(value, 2));
			} else {
				value = Math.pow(value, 2);
			}
		}
		
		return value;
  }

  @Override
  public double getRightX() {
    double value = super.getRightX();

    if(value < precision && value > -precision)
		{
			value = 0;
		}
    
    if(ramp)
		{
			if(value < 0)
			{
				value = -(Math.pow(value, 2));
			} else {
				value = Math.pow(value, 2);
			}
		}
		
		return value;
  }

  @Override
  public double getRightY() {
    double value = super.getRightY();

    if(value < precision && value > -precision)
		{
			value = 0;
		}
    
    if(ramp)
		{
			if(value < 0)
			{
				value = -(Math.pow(value, 2));
			} else {
				value = Math.pow(value, 2);
			}
		}
		
		return value;
  }

  /**
	 * Method that returns whether the X and Y values of the joy sticks are being
	 * ramped parabolically so that it is less sensitive as the sticks are in their natural
	 * positions
	 * @return Returns TRUE if ramp is being applied
	 */
  public boolean getRamp() {
    return this.ramp;
  }

  /**
   * Returns the precision of the controller, used to specify the deadband of the
   * controller
   * @return Double value between 0.0 and 1.0
   */
  public double getPrecision()
	{
		return precision;
	}

  /**
	 * Used to set the deadband of X and Y axes for the joy sticks. Value is range of
	 * 0.0 to 1.0 (zero being no deadband and 1 being evenything is deadband and does nothing).
	 * Suggest starting at value of 0.15
	 * @param value Value to set Deadband to. Range from 0.00 to 1.00
	 */
	public void setPrecision(double value)
	{
		if(value >= 1.0) {
			precision = 1.0;
		} else if (value <= 0) {
      this.precision = 0.0;
    } else {
      this.precision = value;
    }
	}

  /**
   * Rumbles the controller for the supplied "rumbleTimeLength" and will not
   * allow the controller to be rumbled again for the length of the supplied "coolOffPeriod"
   * <p><b>NOTE:</b> Make sure to add this controller's periodic() method to Robot.periodic()
   * for rumble time feature to work!</p>
   * @param power The normalized value (0 to 1) to set the rumble to
   * @param rumbleTimeLength Length of time in seconds
   * @param coolOffPeriodLength Length of time in seconds
   */
  public void rumbleForSetTime(double power, int rumbleTimeLength, int coolOffPeriodLength) {
    if (this.stopRumbleTime == null && this.coolOffRumbleTime == null) {
      this.stopRumbleTime = System.currentTimeMillis() + (1000 * rumbleTimeLength);
      this.coolOffRumbleTime = System.currentTimeMillis() + (1000 * coolOffPeriodLength);
      this.setRumble(RumbleType.kBothRumble, power);
    }
  }

  /**
   * Hanlde the timed rumble feature of the controller. Add to Robot.periodic()
   */
  public void periodic() {
    // Only run if there is a value for stopRumbleTime
    if (this.stopRumbleTime != null) {
      // Check to see if we should stop the rumble
      if (System.currentTimeMillis() > this.stopRumbleTime) {
        this.setRumble(RumbleType.kBothRumble, 0);
        this.stopRumbleTime = null;
      }
    }
      
    // Run only if there is a value for coolOffTimer
    if (this.coolOffRumbleTime != null) {
      // Check to see if the cool off time is up and we reset the
      // the rumble system
      if (System.currentTimeMillis() > this.coolOffRumbleTime) {
        this.coolOffRumbleTime = null;
      }
    }
  }
}

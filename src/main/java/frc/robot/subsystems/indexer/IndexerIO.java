package frc.robot.subsystems.indexer;

public interface IndexerIO {
    // public default void setIndexerPower(double power){} 
    // public default void setIndexWallPower(double power){}
    
    /**
     * Sets the power of the horizontal indexer motors to move
     * fuel from the fuel intake to the kicker. Will used the
     * supplied value and convert it to a value between -12.0
     * volts and 12.0 volts
     * <p>
     * Example use:
     * <pre>{@code 
     * IndexerIO io = new IndexerIOTalonFX();
     * io.setIndexerPower(0.5);
     * }</pre>
     * @param power Power value between -1.0 and 1.0
     */
    public default void setIndexerPower(double power){}; 
    
    /**
     * Sets the power of the vertical indexer motors to move
     * fuel from the right side of the robot to the left side,
     * A.K.A to the kicker.
     * <p>
     * Example use:
     * <pre>{@code 
     * IndexerIO io = new IndexerIOTalonFX();
     * io.setIndexerWallPower(0.5);
     * }</pre>
     * @param power Power value between -1.0 and 1.0
     */
    // public default void setIndexerWallPower(double power){};

    /**
     * Updates the supplied inputs objects with the current status of the 
     * indexer and indexer wall motors
     * @param inputs IndexerIOInputs object
     */
    public default void updateInputs(IndexerIOInputs inputs){};
}

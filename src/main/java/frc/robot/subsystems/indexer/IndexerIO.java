package frc.robot.subsystems.indexer;

public interface IndexerIO {
    // public default void setIndexerPower(double power){} 
    // public default void setIndexWallPower(double power){}
    public default void setIndexerPower(double power){}; 
    public default void setIndexWallPower(double power){};

    public default void updateInputs(IndexerIOInputs inputs){};
}

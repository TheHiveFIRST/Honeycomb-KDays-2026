package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

//TODO: add comments 
public class IntakeConfig {
  //Intake motor config
  public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
  static {
    intakeMotorConfig.smartCurrentLimit(80);
    }

  //Arm config
  public static final class ArmConfig{
    public static final SparkMaxConfig pivotLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotFollowerConfig = new SparkMaxConfig();
    static {
        //configuration for leader and follower motor 
        pivotLeaderConfig.idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(50);;
        
        pivotFollowerConfig.idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(50);;
    }
  }
}

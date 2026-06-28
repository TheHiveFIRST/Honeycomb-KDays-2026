package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ShooterConfig{
        public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFeederLeaderConfig = new SparkMaxConfig();
        public static final SparkMaxConfig shooterFeederFollowerConfig = new SparkMaxConfig();

        static {
            // Leader configuration
            shooterLeaderConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kCoast);
            // Follower configuration - set to follow Leader and invert
            shooterFollowerConfig
                .inverted(true)
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kCoast);

            // Feeder/Kicker configuration
            shooterFeederLeaderConfig
                .smartCurrentLimit(50);
            shooterFeederFollowerConfig
                .smartCurrentLimit(50);
        }
}
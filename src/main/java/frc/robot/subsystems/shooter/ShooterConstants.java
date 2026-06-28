package frc.robot.subsystems.shooter;

public final class ShooterConstants {
            public static final int SHOOTER_LEADER_CANID = 14;
        public static final int SHOOTER_FOLLOWER_CANID = 15;
        public static final int SHOOTER_FEEDER_LEADER_CANID = 16; 
        public static final int SHOOTER_FEEDER_FOLLOWER_CANID = 17;

        // PIDF Values
        public static final double LEADER_Kp = 0.0000709999;
        public static final double LEADER_Kd = 0; 
        public static final double LEADER_Ki = 0; 
        //feedforward controller values 
        public static final double LEADER_FF_kS = 0.0;
        public static final double LEADER_FF_kV = 0.00012;
        public static final double LEADER_FF_kA = 0.0002;

        public static final double FF_KS = 0.0;
        public static final double FF_KV = 0.0;
        public static final double FF_KA = 0.0;
    
        //PID Tuning Increments 
        public static final double RPM_INCREMENT = 12.5;
        public static final double KV_INCREMENT = 0.000001;
        public static final double KP_INCREMENT = 0.001;
        public static final double KI_INCREMENT = 0.0001;
        public static final double KD_INCREMENT = 0.0001;

        //Manual control speeds 
        public static final double KICKER_SPEED = 1.0;
        public static final double KICKER_REVERSED_SPEED = -1.0;
        public static final double SHOOTER_SPEED = 0.7; 
        public static final double RPM_OFFSET_INCREMENT = 200; 
        public static final double VELOCITY_TOLERANCE =  30; 


        //Set RPM for certain distances to Hub 
        public static final double HUB_TARGET_RPM = 4500; 
        public static final double TRENCH_TARGET_RPM = 6800; 
        public static final double LADDER_TARGET_RPM = 6300;
        public static final double PASSING_TARGET_RPM = 8000;
        public static final double DEFENCE_TARGET_RPM = 5200; 

        public static final double AUTO_TARGET_RPM = 5400; 


        //Regression coefficients 
        public static final double REGRESSION_COEFFICIENT_4 = 0; 
        public static final double REGRESSION_COEFFICIENT_3 = 0;
        public static final double REGRESSION_COEFFICIENT_2 = -0;
        public static final double REGRESSION_COEFFICIENT_1 = 670;
        public static final double REGRESSION_COEFFICIENT_0 = 3520;

        //untested kinematics equation constants 
        public static final double LAUNCH_ANGLE_RAD = Math.toRadians(25.0);  
        public static final double HEIGHT_DIFF_METERS = 1.296867993;                 
        public static final double WHEEL_RADIUS_METERS = 0.0508;          
        public static final double LAUNCH_EFFICIENCY = 0.85;                 // 0.80–0.95, TODO: tune launch efficiency
    
}

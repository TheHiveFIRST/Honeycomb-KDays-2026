package frc.robot.subsystems.intake;
 /**
   * Intake related constants. 
   */

   //TODO: add comments 

public final class IntakeConstants {

    //Intake motor constants 
    public static final int INTAKE_MOTOR_ID = 20;
    public static final double INTAKE_SPEED = -1.0; // full forward
    public static final double SLOW_INTAKE_SPEED = -0.2; // slower for gentle intake
    public static final double OUTTAKE_SPEED = 0.5; // reverse

    //Arm Pivot Constnats 
    public static final class ArmConstants {
      public static final int ARM_LEADER_ID = 12;
      public static final int ARM_FOLLOWER_ID = 13;

      //PID Gains
      public static final double ARM_KP = 2; 
      public static final double ARM_KI = 0.0;
      public static final double ARM_KD = 0.0;
      public static final double ARM_KP_INCREMENT = 0.01;

      public static final double PIVOT_OUT = 0.73;
      public static final double PIVOT_BUMP = 0.45;  
      public static final double PIVOT_AGITATE = 0.65;
      public static final double PIVOT_AGITATE55 = 0.55;
      public static final double PIVOT_AGITATE45 = 0.45;
      public static final double PIVOT_AGITATE35 = 0.35;
      public static final double PIVOT_IN = 0.24;  
     
      public static final double POSITION_TOLERANCE = 0.1;
    }
}



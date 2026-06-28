package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
  private final SparkMax mShooterLeader;
  private final SparkMax mShooterFollower;
  private final SparkMax mKickerLeader;
  private final SparkMax mKickerFollower;
  private final RelativeEncoder mShooterLeaderEncoder;
  private final RelativeEncoder mShooterFollowerEncoder; 
  private final PIDController mShooterPID;
private final SimpleMotorFeedforward tempFF;


//starting RPM values on initialization
private double mTargetRPM = ShooterConstants.HUB_TARGET_RPM;
private double mKinematicsRPM = 0; 
private double mShooterRPMOffset = 0; 

//TODO: add a isAtSpeed method to allow shots only when up to speed 

//toggable state variables at match start  
private boolean mDistanceEstimationEnabled = true;
private boolean mShooterEnabled = false;
private String shotType = "HUB_SHOT";

//sysID 
private final MutVoltage m_appliedVoltage  = Volts.mutable(0);
private final MutAngle m_encoderAngle    = Rotations.mutable(0);
private final MutAngularVelocity m_encoderVelocity = RotationsPerSecond.mutable(0);
private final SysIdRoutine mSysIdRoutine;

   

    public Shooter() {
      mShooterLeader = new SparkMax(ShooterConstants.SHOOTER_LEADER_CANID, MotorType.kBrushless);
        mShooterFollower = new SparkMax(ShooterConstants.SHOOTER_FOLLOWER_CANID, MotorType.kBrushless);
        mKickerLeader = new SparkMax(ShooterConstants.SHOOTER_FEEDER_LEADER_CANID, MotorType.kBrushless);
        mKickerFollower = new SparkMax(ShooterConstants.SHOOTER_FEEDER_FOLLOWER_CANID, MotorType.kBrushless);


        mShooterLeader.configure(ShooterConfig.shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mShooterFollower.configure(ShooterConfig.shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mKickerLeader.configure(ShooterConfig.shooterFeederLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mKickerFollower.configure(ShooterConfig.shooterFeederFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        mShooterLeaderEncoder = mShooterLeader.getEncoder();
        mShooterFollowerEncoder = mShooterFollower.getEncoder();

        mShooterPID = new PIDController(ShooterConstants.LEADER_Kp, 
        ShooterConstants.LEADER_Ki, ShooterConstants.LEADER_Kd);
        
        tempFF = new SimpleMotorFeedforward(ShooterConstants.LEADER_FF_kS,
        ShooterConstants.LEADER_FF_kV, 
        ShooterConstants.LEADER_FF_kA);

        mSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Second),   // ramp rate  – 1 V/s (quasistatic)
                Volts.of(6),               // step voltage – 4 V  (dynamic)
                Seconds.of(5)              // timeout per direction
            ),
            new SysIdRoutine.Mechanism(
                // Drive: apply a raw voltage to both shooter motors
                voltage -> {
                    double v = voltage.in(Volts);
                    mShooterLeader  .setVoltage(v);
                    mShooterFollower.setVoltage(v);
                },
                // Log: record voltage, position, and velocity for the leader
                log -> {
                    log.motor("shooter-leader")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                mShooterLeader.getBusVoltage()
                                    * mShooterLeader.getAppliedOutput(),
                                Volts))
                        .angularPosition(
                            m_encoderAngle.mut_replace(
                                mShooterLeaderEncoder.getPosition(),
                                Rotations))
                        .angularVelocity(
                            m_encoderVelocity.mut_replace(
                                // Encoder reports RPM – convert to RPS for WPILib units
                                mShooterLeaderEncoder.getVelocity() / 60.0,
                                RotationsPerSecond));
                },
                this   // owning subsystem (for requirement tracking)
            )
        );

    }

    // Methods

    /**
     * Main PID+feedforward controller that computes and applies motor output.
     *
     * @param setRPM desired shooter speed (RPM)
     * @param RPMOffset offset to add to the desired RPM
     */
    public void setShooterSpeeds(double setRPM, double RPMOffset) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset);
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        runShooterPower(motorPower);
    }

    /**
     * Compute a target RPM from a polynomial regression based on distance and
     * update the internal target RPM. Falls back to a safe RPM if the
     * regression suggests an unrealistically low value.
     *
     * @param distance distance in meters
     */
    public void runShooterRegression(double distance) {
        double shooterRegressionRPM =
            (Math.pow(distance, 3) * ShooterConstants.REGRESSION_COEFFICIENT_3)
                + (Math.pow(distance, 2) * ShooterConstants.REGRESSION_COEFFICIENT_2)
                + (Math.pow(distance, 1) * ShooterConstants.REGRESSION_COEFFICIENT_1)
                + (ShooterConstants.REGRESSION_COEFFICIENT_0);
        if (shooterRegressionRPM <= ShooterConstants.HUB_TARGET_RPM) {
            // Distance is too short; fall back
            updateRPM(ShooterConstants.HUB_TARGET_RPM);
            return;
        }
        updateRPM(shooterRegressionRPM);
    }

    /**
     * Compute required wheel RPM from kinematic projectile motion equations and
     * update the target RPM. Falls back for invalid geometry.
     *
     * @param distanceMeters horizontal distance to target in meters
     */
    public void runShooterKinematics(double distanceMeters) {
        double theta = ShooterConstants.LAUNCH_ANGLE_RAD;
        double deltaH = ShooterConstants.HEIGHT_DIFF_METERS;
        double r = ShooterConstants.WHEEL_RADIUS_METERS;
        double eta = ShooterConstants.LAUNCH_EFFICIENCY;
        double gravityConstant = 9.81;

        double tanTheta = Math.tan(theta);
        double cosTheta = Math.cos(theta);
        double denom = 2.0 * cosTheta * cosTheta * (distanceMeters * tanTheta - deltaH);

        if (denom <= 0) {
            // Distance is too short for this angle/height combo
            updateRPM(ShooterConstants.HUB_TARGET_RPM);
            return;
        }

        double v0 = Math.sqrt((gravityConstant * distanceMeters * distanceMeters) / denom);

        // Surface speed of wheel = v0 / eta, convert to RPM
        mKinematicsRPM = (v0 / (eta * r)) * (60.0 / (2.0 * Math.PI));

        updateRPM(mKinematicsRPM);
    }

    /**
     * Return the most recently computed kinematics RPM.
     *
     * @return last computed kinematics RPM
     */
    public double getKinematicsRPM() {
        return mKinematicsRPM;
    }

    /**
     * Directly set shooter motor power (open-loop).
     *
     * @param motorPower motor power in range [-1.0, 1.0]
     */
    public void runShooterPower(double motorPower) {
        mShooterLeader.set(motorPower);
        mShooterFollower.set(motorPower);
    }

    /**
     * Stop shooter motors and kicker.
     */
    public void stopShooterKicker() {
        runShooterPower(0);
        runKicker(0);
    }

    /**
     * Run kicker (feeder) motors. Follower is inverted relative to leader.
     *
     * @param speed motor speed in range [-1.0, 1.0]
     */
    public void runKicker(double speed) {
        mKickerLeader.set(speed);
        mKickerFollower.set(-1 * speed);
    }

    /**
     * Get the average velocity of both shooter encoders (RPM).
     *
     * @return average RPM
     */
    public double getAverageVelocity() {
        double sum = mShooterLeaderEncoder.getVelocity() + mShooterFollowerEncoder.getVelocity();
        double average = sum / 2;
        return average;
    }

    /**
     * Return the motor output computed by the PID+feedforward controller without
     * applying it.
     *
     * @param setRPM desired RPM
     * @param RPMOffset offset to add to desired RPM
     * @return clipped motor output in [0,1]
     */
    public double getShooterPIDF(double setRPM, double RPMOffset) {
        double mCurrentRPM = mShooterLeaderEncoder.getVelocity();
        double pidOutput = mShooterPID.calculate(mCurrentRPM, setRPM + RPMOffset);
        double ffOutput = tempFF.calculate(setRPM + RPMOffset);
        double motorPower = MathUtil.clamp(pidOutput + ffOutput, 0.0, 1.0);
        return motorPower;
    }

    // Offsets and updates
    /**
     * Change the shooter RPM tuning offset.
     *
     * @param amount amount to change the offset by
     */
    private void changeShootingRPMOffset(double amount) {
        mShooterRPMOffset += amount;
    }

    /**
     * Update the internal target RPM.
     *
     * @param newRPM new target RPM
     */
    //TODO: make it not self referiental 
    public void updateRPM(double newRPM) {
        mTargetRPM = newRPM;
    }

    public void updateRPMs(){

    }

    /** Increase target RPM by configured increment */
    public void incrementRPM() {
        mTargetRPM += ShooterConstants.RPM_INCREMENT;
    }

    /** Decrease target RPM by configured increment */
    public void decrementRPM() {
        mTargetRPM -= ShooterConstants.RPM_INCREMENT;
    }

   

    //Commands 
    //TODO: change the logic flow so that there are less commands, MOVE TO ShootCommands following convention and centralized method for setting RPM input
    public Command runShooterAutoCommand() {       
        return run(
        () -> {
            setShooterSpeeds(ShooterConstants.AUTO_TARGET_RPM, 0);
              }); 
        }
    
    public Command runShooterPIDFCommand() {
         return run(
        () -> {
            setShooterSpeeds(mTargetRPM, mShooterRPMOffset);
              });
    }
     public Command runShooterRegressionCommand() {
         return run(
        () -> {
            runShooterRegression(DriveSubsystem.hubDistance);
              });
    }
    public Command runShooterKinematicsCommand() {
         return run(
        () -> {
            runShooterKinematics(DriveSubsystem.hubDistance);
              });
    }
    public Command runShooterPowerCommand() {
         return run(
        () -> {
            runShooterPower(ShooterConstants.SHOOTER_SPEED);
              });
    }

    //kicker commands
    public Command runKickerCommand() {
         return run(
        () -> {
            runKicker(-ShooterConstants.KICKER_SPEED);
        
              });
    }

    public Command runKickerBackwardCommand() {
         return run(
        () -> {
            runKicker(ShooterConstants.KICKER_SPEED);
              });
    }

    //stops all motors 
    public Command stop() {
         return run(
        () -> {
            stopShooterKicker();
              });
    }



    //toggle for operator control and continous input to PID controller in setShooterSpeeds
    public Command toggleShooterCommand() {
        return new InstantCommand(() -> mShooterEnabled = !mShooterEnabled);}

    public Command toggleDistanceEstimationCommand() {
        return new InstantCommand(() -> mDistanceEstimationEnabled = !mDistanceEstimationEnabled);}
    
    public Command toggleAutoShooterCommand() {
       return new InstantCommand(() -> mShooterEnabled = true);}
    
    public Command toggleOffAutoShooterCommand() {
       return new InstantCommand(() -> mShooterEnabled = false);}
    
    
    public Command increaseShootingRPMOffsetCommand(){
    return new InstantCommand(() -> changeShootingRPMOffset(ShooterConstants.RPM_OFFSET_INCREMENT));
    }
    public Command decreaseShootingRPMOffsetCommand(){
      return new InstantCommand(() -> changeShootingRPMOffset(-ShooterConstants.RPM_OFFSET_INCREMENT));
    }

    //manual shot settings from operator control
    public Command setHubShotCommand() {
    return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.HUB_TARGET_RPM;
        mShooterRPMOffset = 0;
        mDistanceEstimationEnabled = false; 
        shotType = "BUMPER_ALIGN_SHOT";
        });
    }
    public Command setAutoShotCommand() {
    return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.AUTO_TARGET_RPM;
        mShooterRPMOffset = 0;
        shotType = "AUTOSHOT";
        });
    }

    public Command setTrenchShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.TRENCH_TARGET_RPM;
        mShooterRPMOffset = 0;
        mDistanceEstimationEnabled = false; 
        shotType = "TRENCH_SHOT";
        });
    }
    
    public Command setDefenceShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.DEFENCE_TARGET_RPM;
        mShooterRPMOffset = 0;
        mDistanceEstimationEnabled = false; 
        shotType = "DEFENCE_SHOT";
        });
    }

    public Command setLadderShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.LADDER_TARGET_RPM;
        mShooterRPMOffset = 0;
        mDistanceEstimationEnabled = false; 
        shotType = "TOWER_SHOT";
        });
    }
    
    public Command setPassingShotCommand() {
          return new InstantCommand(() -> {
        mTargetRPM = ShooterConstants.PASSING_TARGET_RPM;
        mShooterRPMOffset = 0;
        mDistanceEstimationEnabled = false; 
        shotType = "PASSING_SHOT";
        });
    }



    //sysID tests 
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return mSysIdRoutine.quasistatic(direction);
    }
    /**
     * Dynamic test – applies a voltage step.
     * Used to fit kA.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return mSysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {

        //TODO: move these into seperate method that updates here 
        if (mDistanceEstimationEnabled) {
            runShooterRegression(DriveSubsystem.hubDistance);
        } else {
            updateRPM(mTargetRPM);
        }

        if (mShooterEnabled) {
            setShooterSpeeds(mTargetRPM, mShooterRPMOffset);
        } else {
            runShooterPower(0);
        }

        boolean atSpeed = Math.abs(mShooterLeaderEncoder.getVelocity() - (mTargetRPM + mShooterRPMOffset)) < ShooterConstants.VELOCITY_TOLERANCE;
        SmartDashboard.putNumber("Shooter/Target RPM", mTargetRPM + mShooterRPMOffset);
        SmartDashboard.putNumber("Shooter/Actual RPM", mShooterLeaderEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter/RPM Offset", mShooterRPMOffset);
        SmartDashboard.putString("Shooter/Shot Type", shotType);
        SmartDashboard.putBoolean("Shooter/Shooter Ready", atSpeed);
        SmartDashboard.putBoolean("Shooter/Shooter Toggled", mShooterEnabled);
        SmartDashboard.putBoolean("Shooter/Regression Toggled", mDistanceEstimationEnabled);
        SmartDashboard.putNumber("Shooter/Kinematics RPM", getKinematicsRPM());
    }



    // Tuning  
    // 
    //public enum TuningMode {
    //     KP, KV, KD, 
    // }
    // private TuningMode mCurrentTuningMode = TuningMode.KV;
    // // Cycle through tuning modes
    // public void cycleTuningMode() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             mCurrentTuningMode = TuningMode.KP;
    //             break;
    //         case KV:
    //             mCurrentTuningMode = TuningMode.KV;
    //             break;
    //         case KD:
    //             mCurrentTuningMode = TuningMode.KD;
    //             break;
    //     }
    // }
    //     public void incrementCurrentGain() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             incrementKP();
    //             break;
    //         case KV:
    //             incrementKV();
    //             break;
    //         case KD:
    //             incrementKD();
    //             break;
    //     }
    // }
    //     public void decrementCurrentGain() {
    //     switch (mCurrentTuningMode) {
    //         case KP:
    //             decrementKP();
    //             break;
    //         case KV:
    //             decrementKV();
    //             break;
    //         case KD:
    //             decrementKD();
    //             break;
    //     }
    // }

    // public void incrementKD() { mCurrentKD += ShooterConstants.KD_INCREMENT; } 
    // public void decrementKD() { mCurrentKD -= ShooterConstants.KD_INCREMENT; }

    // public void incrementKI() { mCurrentKI += ShooterConstants.KI_INCREMENT; } 
    // public void decrementKI() { mCurrentKI -= ShooterConstants.KI_INCREMENT; }

    // public void incrementKP() { mCurrentKP += ShooterConstants.KP_INCREMENT; } 
    // public void decrementKP() { mCurrentKP -= ShooterConstants.KP_INCREMENT; }

    // public void incrementKV() { mCurrentKV += ShooterConstants.KV_INCREMENT; } 
    // public void decrementKV() { mCurrentKV -= ShooterConstants.KV_INCREMENT; }
}
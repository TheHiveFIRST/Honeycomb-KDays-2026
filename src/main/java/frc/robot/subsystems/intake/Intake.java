package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
  private final SparkFlex mIntakeMotor;

  //TODO: ADD comments 
  public Intake() {
    mIntakeMotor = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    mIntakeMotor.configure(IntakeConfig.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Methods
  public void stopIntake() {
    runIntake(0.0);
  }

  public void runIntake(double speed) {
    mIntakeMotor.set(speed);
  }

  // Command factories
  public Command runIntakeForwardCommand() {
    return Commands.run(() -> runIntake(IntakeConstants.INTAKE_SPEED), this);
  }

  public Command runIntakeSlowCommand() {
    return Commands.run(() -> runIntake(IntakeConstants.SLOW_INTAKE_SPEED), this);
  }

  public Command runOuttakeCommand() {
    return Commands.run(() -> runIntake(IntakeConstants.OUTTAKE_SPEED), this);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(this::stopIntake, this);
  }

  @Override
  public void periodic() {
    // EX: SmartDashboard.putNumber("Intake/AppliedVolts", mIntakeMotor.getAppliedOutput());
  }
}

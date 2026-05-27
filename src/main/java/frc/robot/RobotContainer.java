// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.sim.FuelSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private SwerveDriveSimulation driveSimulation = null;
  public static FuelSim fuelSim = new FuelSim("fuel");

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Keyboard for sim
  private final CommandGenericHID keyboard = new CommandGenericHID(1); // Keyboard on port 1

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(), // switched to NavX with USB import
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});
        this.vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        fuelSim.enableAirResistance();
        fuelSim.start();

        // fuelSim.registerRobot(
        //     Constants.BUMPER_WIDTH,
        //     Constants.BUMPER_WIDTH,
        //     Inches.of(6),
        //     () -> drive.getRobotPose(),
        //     () -> drive.getFieldRelativeSpeeds()
        // );

        fuelSim.registerIntake(
            Inches.of(15), Inches.of(22), Inches.of(-15), Inches.of(15), () -> true, () -> {});

        fuelSim.spawnStartingFuel();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Check if we should use keyboard for input in sim
    boolean useKeyboard =
        Constants.currentMode == Constants.Mode.SIM && Constants.USE_KEYBOARD_FOR_SIM;

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> useKeyboard ? keyboard.getRawAxis(1) : -controller.getLeftY(),
            () -> useKeyboard ? keyboard.getRawAxis(0) : -controller.getLeftX(),
            () -> useKeyboard ? keyboard.getRawAxis(2) : -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> useKeyboard ? keyboard.getRawAxis(1) : -controller.getLeftY(),
                () -> useKeyboard ? keyboard.getRawAxis(0) : -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Auto aim command example
    @SuppressWarnings("resource")
    PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> aimController.calculate(vision.getTargetX(0).getRadians())));

    // Example Coral Placement Code
    // TODO: delete these code for your own project
    if (Constants.currentMode == Constants.Mode.SIM) {
      // L4 placement
      controller
          .y()
          .onTrue(
              Commands.runOnce(
                  () ->
                      SimulatedArena.getInstance()
                          .addGamePieceProjectile(
                              new ReefscapeCoralOnFly(
                                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                  new Translation2d(0.4, 0),
                                  driveSimulation
                                      .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                  driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                  Meters.of(2),
                                  MetersPerSecond.of(1.5),
                                  Degrees.of(-80)))));
      // L3 placement
      controller
          .b()
          .onTrue(
              Commands.runOnce(
                  () ->
                      SimulatedArena.getInstance()
                          .addGamePieceProjectile(
                              new ReefscapeCoralOnFly(
                                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                  new Translation2d(0.4, 0),
                                  driveSimulation
                                      .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                  driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                  Meters.of(1.35),
                                  MetersPerSecond.of(1.5),
                                  Degrees.of(-60)))));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
    SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(2, 3)));
  }

  public void updateSimulationToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("ferry1", new Translation2d(2, 7));

    // Logger.recordOutput("ferry2",new Translation2d(2,1));
    // Logger.recordOutput("ferrydistance", turret.getDistanceToPoint(new Translation2d(2,7)));
    // TODO: add shooter
  }
}

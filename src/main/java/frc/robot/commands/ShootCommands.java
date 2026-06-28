package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GameObjectConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommands {

  
  
  
    public static Command runShooterRegressionCommand(Drive drive, Shooter shooter ){
    double hubShotDistance = getShotDistance(
    GameObjectConstants.getHubPoseTranslation2D(), drive);
    return Commands.run(
        () -> {
            shooter.runShooterRegression(hubShotDistance);
        }, drive);
  }


  //TODO: THIS CANT BE STATIC MUST CHANGE ALL THE TIME JUST DO MANUAL SHOT FOR NOW 
    public static double getShotDistance(Translation2d targetPose, Drive drive){
      Pose2d odometryPose = drive.getPose();
      double shooterToTargetMeters = odometryPose.getTranslation().getDistance(targetPose);
      return shooterToTargetMeters; 
   }
}

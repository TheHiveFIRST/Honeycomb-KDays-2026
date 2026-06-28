package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameObjectConstants{
   
    //returns the current alliance of the match 
    public static Alliance getCurrentAlliance() {
    return DriverStation.getAlliance().get();
    }
    
    //returns the position of the hub based on alliance 
    public static final Pose3d getHubPose() {
      Pose3d pose; 
      if (getCurrentAlliance() == Alliance.Blue) {
         pose = DriveConstants.BlueHubPose; 
      } else {
         pose = DriveConstants.RedHubPose; 
      }      
      return pose;
    }

    //returns the hubpose as a Translation 2D object
    public static Translation2d getHubPoseTranslation2D(){
      return getHubPose().toPose2d().getTranslation();
    }
}

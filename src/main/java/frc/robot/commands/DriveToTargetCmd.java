package frc.robot.commands;


import frc.robot.subsystems.VisionTestSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Collections;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveToTargetCmd extends Command{

    SwerveSubsystem m_swerveSubsystem;

    VisionTestSubsystem m_visionSubsys;

    HashMap<String, Double> camData = new HashMap<String, Double>();

    double finalDist = 1;
    double finalAngle = 0;
    double finalY = 0;


    public DriveToTargetCmd(SwerveSubsystem swerve, VisionTestSubsystem vision){
        m_swerveSubsystem = swerve;
        m_visionSubsys = vision;

    }


    
    public Rotation2d GetRadian(){
        Rotation2d radians = new Rotation2d(camData.get("targetposeYaw") * Math.PI/180);

        return radians;
    }
       
    private void getCamData(){

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //set getTable() to limelight name NOT pipeline name

    double[] targetpose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    double targetposeY = targetpose[1];
    double targetposeZ = targetpose[3];
    double targetposeYaw = targetpose[5];
    
    camData.put("targetposeY", targetposeY);
    camData.put("targetposeZ", targetposeZ);
    camData.put("targetposeYaw", targetposeYaw);
    }
    @Override 
    public void execute(){
        
    }
    @Override
    public void initialize(){
    
    getCamData();

        TrajectoryConfig  moveConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                        (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                        .setKinematics(SDC.SWERVE_KINEMATICS);
        
        moveConfig.setReversed(false);
        var start = new Pose2d(camData.get("targetposeZ"),
                               camData.get("targetposeY"),
                               GetRadian());

        var end = new Pose2d(finalDist,
                             finalY,
                             new Rotation2d(finalAngle));
        
        TrajectoryGenerator.generateTrajectory(
        start,
        end,
        moveConfig
        );
    }

   /* 
    TrajectoryGenerator.generateTrajectory
    (
    new Pose2d(Units.inchesToMeters(41.0), 
               Units.inchesToMeters(284.0), 
               Rotation2d.fromDegrees(60.0)),
     List.of(new Translation2d(Units.inchesToMeters(42.0), 
                               Units.inchesToMeters(285.0)),
             new Translation2d(Units.inchesToMeters(55.0), 
                               Units.inchesToMeters(304.0)),
             new Translation2d(Units.inchesToMeters(165.0), 
                               Units.inchesToMeters(305.0))),
     new Pose2d(Units.inchesToMeters(169.0),
                Units.inchesToMeters(306.0),
                Rotation2d.fromDegrees(0.0)),
     exitConfig
    );*/
    //TODO: icorporate trajectory generator
    

    @Override
    public boolean isFinished(){

        boolean Z = camData.get("targetposeZ") == finalDist;
        boolean Y = camData.get("targetposeY") == finalY;
        boolean Ang = camData.get("targetposeYaw") == finalAngle;

        return Ang && Y && Z;
    }
}

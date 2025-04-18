package frc.robot.subsystems;

import frc.robot.subsystems.SwerveSubsystem;
import us.hebi.quickbuf.RepeatedString;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.hal.can.CANMessageNotFoundException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants.AutoC;
import frc.robot.Constants.GC;
import frc.robot.Constants.SDC;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.EnumNamingStrategies.CamelCaseStrategy;


public class VisionTestSubsystem extends SubsystemBase{
    

    // private final PhotonCamera m_photonCamera;
    // private final LimelightResults m_limelight;
    private final SwerveSubsystem m_swerveSubsystem;
    private final SwerveDrivePoseEstimator m_drivePoseEstimator;
    private final Pose2d initialPoseMeters;
    private final Transform3d CAMERA_TO_ROBOT;
    private final Pigeon2 m_gyro;
    //private final RepeatCommand m_RepeatCommand;
    
    public HashMap<String, Double> camData = new HashMap<String, Double>();

    public double counter = 1;
    private int I = 1;
    //HashMap<String, Double> cam2Data = new HashMap<String, Double>();

    //PhotonCamera camera = new PhotonCamera("limelight");

    

   public VisionTestSubsystem(LimelightResults vision, SwerveSubsystem swerve){
        //this.m_photonCamera = m_photonCamera;
        
        m_swerveSubsystem = swerve;

        initialPoseMeters = m_swerveSubsystem.getPose();

        SmartDashboard.putString("test", "test");

        m_drivePoseEstimator = new SwerveDrivePoseEstimator(
            SDC.SWERVE_KINEMATICS,
            m_swerveSubsystem.getYaw2d(),
            m_swerveSubsystem.getModulePositions(),
            initialPoseMeters
        );

        //m_RepeatCommand = Command.RepeatCommand();
        


    
    CAMERA_TO_ROBOT = new Transform3d();
        m_gyro = new Pigeon2(GC.PIGEON_2_CANID, Constants.ROBO_RIO_BUS_NAME);

    } 
    
    
    public void getVisionData(){

         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //set getTable() to limelight name NOT pipeline name

    // check for target
     long Tv = (table.getEntry("tv").getInteger(0));
        camData.put("Tv", ((double)Tv));
        SmartDashboard.putNumber("Cam has targets", Tv);

    //SmartDashboard.getBoolean("Cam2 has targets", LimelightHelpers.getTV("cam2"));

    
    // get pipeline type
    String pipeType = LimelightHelpers.getCurrentPipelineType("limelight");
        SmartDashboard.putString("pipeType", pipeType);

    // get pipeline number
    double pipeNum = LimelightHelpers.getCurrentPipelineIndex("limelight");
        SmartDashboard.putNumber("pipeType", pipeNum);

    // get fid ID
    double fidID = LimelightHelpers.getFiducialID("limelight");
        SmartDashboard.putNumber("fidID", fidID);

        //get horizontal value
        double Tx = table.getEntry("tx").getDouble(0);
            camData.put("Tx", Tx);
            SmartDashboard.putNumber("TX", Tx);

        //get vertical value
        double Ty = table.getEntry("ty").getDouble(0);
            camData.put("Ty", Ty);
            SmartDashboard.putNumber("TY", Ty);

        //get area value
        double Ta = table.getEntry("ta").getDouble(0);
            camData.put("Ta", Ta);
            SmartDashboard.putNumber("TA", Ta);
        
        //get botpose array
        // note botpose does not give values relative to target
        double[] Botpose = table.getEntry("botpose").getDoubleArray(new double[6]);

        double BotposeX = Botpose[0];
        double BotposeY = Botpose[1];
        double BotposeZ = Botpose[2];

        camData.put("BotposeX", BotposeX);
        camData.put("BotposeY", BotposeY);
        camData.put("BotposeZ", BotposeZ);
        
       SmartDashboard.putNumberArray("Botpose", Botpose);


        // get targetpose_cameraspace data
        // this is relative to the target
        // recieved values are in meters
        double[] targetpose = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        double targetposeX = targetpose[0];
        double targetposeY = targetpose[1];
        double targetposeZ = targetpose[2];

        double targetposeRoll = targetpose[3];
        double targetposePitch = targetpose[4];
        double targetposeYaw = targetpose[5];
/*
        targetposeX *= 39.3701; //convert values to inches from meters
        targetposeY *= 39.3701; 
        targetposeZ *= 39.3701; 
*/
        camData.put("targetposeX", targetposeX);
        camData.put("targetposeY", targetposeY);
        camData.put("targetposeZ", targetposeZ);

        camData.put("targetposeRoll", targetposeRoll);
        camData.put("targetposePitch", targetposePitch);
        camData.put("targetposeYaw", targetposeYaw);

        SmartDashboard.putNumber("targetposeX", targetposeX);
        SmartDashboard.putNumber("targetposeY", targetposeY);
        SmartDashboard.putNumber("targetposeZ", targetposeZ);

        SmartDashboard.putNumber("targetposeRoll", targetposeRoll);
        SmartDashboard.putNumber("targetposePitch", targetposePitch);
        SmartDashboard.putNumber("targetposeYaw", targetposeYaw);

        
        SmartDashboard.putNumberArray("targetpose", targetpose);

        double distanceStraightLine = Math.sqrt(targetposeX*targetposeX) +
                                               (targetposeY*targetposeY) + 
                                               (targetposeZ*targetposeZ);

        camData.put("distanceStraightLine", distanceStraightLine);

        SmartDashboard.putNumber("distanceStraightLine", distanceStraightLine); 
    
    }

    


    
    @Override
    public void periodic(){ 

        //Yaw = camera.getAllUnreadResults().get(0);
        //SmartDashboard.putBoolean("limelight Has Targets", camera.getLatestResult().hasTargets());
        getVisionData();

        getCamAreaDist();

        getTargetposeDist();
    }

    public double aimProportional(){

        double kp = -.055;

        double targetingAngularVelocity = MathUtil.applyDeadband((camData.get("Tx")* kp), 0);

        targetingAngularVelocity *= Constants.SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC;
        return targetingAngularVelocity;

    }


    public double getCamAreaDist(){

        double KP = .01165; //calculated based on testing num too small-increase value too big-decrease value

        //calculate distance based on actual area and area % gotten by camera
        double distanceArea = Math.sqrt(Constants.VC.APRILTAG_AREA_IN/ (camData.get("Ta") * KP));


        camData.put("distanceArea", distanceArea);
        SmartDashboard.putNumber("distArea", distanceArea);

        return distanceArea;
    }

    public double getCamAngleDist(){

        double distanceAngle = (Constants.VC.PROCESSOR_HEIGHT-Constants.VC.CAM_HEIGHT) / Math.tan(I);

        camData.put("distAngle", distanceAngle); //post value to camData table
        SmartDashboard.putNumber("distAngle", distanceAngle); //post value to smart dashboard

        return distanceAngle;
    }

    public double getTargetposeDist(){
       

        return camData.get("distanceStraightLine");
    }

    public Translation2d targetLocation(){
        //todo: make distance aquisition type switch between area and angle based on angle
        double strafeKp = 0.2;

        double targetDistToTravel = MathUtil.applyDeadband(getCamAreaDist(), .2); //get area-based distance

        double targetX = -MathUtil.applyDeadband(camData.get("Tx"), .2); //get x value for target apply deadband
        
        if (targetDistToTravel == 30){targetDistToTravel = 0.0;}
        targetX *= strafeKp;

        Translation2d translation = new Translation2d(-targetDistToTravel, targetX);

        return translation;
    }

    

    public Translation2d testTranslate(){

        Translation2d testTranslate = new Translation2d(0,0);

        return testTranslate;
    }

    //public 
/*
    public void execute(){
        SmartDashboard.putString("test", "stuff");

        m_swerveSubsystem.drive(targetLocation(), aimProportional(), false);

        counter ++;
        SmartDashboard.putNumber("counter", counter);
        
    }
*/
}
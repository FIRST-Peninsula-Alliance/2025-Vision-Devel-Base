package frc.robot.subsystems;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GC;
import frc.robot.Constants.SDC;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.HashMap;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.EnumNamingStrategies.CamelCaseStrategy;


public class VisionTestSubsystem extends SubsystemBase{

    //private final PhotonCamera m_photonCamera;
    //private final LimelightResults m_limelight;
    private final SwerveSubsystem m_swerveSubsystem;
    private final SwerveDrivePoseEstimator m_drivePoseEstimator;
    private final Pose2d initialPoseMeters;
    private final Transform3d CAMERA_TO_ROBOT;
    private final Pigeon2 m_gyro;
    
    HashMap<String, Double> camData = new HashMap<String, Double>();

    //HashMap<String, Double> cam2Data = new HashMap<String, Double>();

    //PhotonCamera camera = new PhotonCamera("limelight");



    

   public VisionTestSubsystem(LimelightResults m_limelight, SwerveSubsystem m_SwerveSubsystem){
        //this.m_photonCamera = m_photonCamera;
        
        m_swerveSubsystem = m_SwerveSubsystem;

        initialPoseMeters = m_swerveSubsystem.getPose();

        SmartDashboard.putString("test", "test");

        m_drivePoseEstimator = new SwerveDrivePoseEstimator(
            SDC.SWERVE_KINEMATICS,
            m_swerveSubsystem.getYaw2d(),
            m_swerveSubsystem.getModulePositions(),
            initialPoseMeters
        );
        


    CAMERA_TO_ROBOT = new Transform3d();
        m_gyro = new Pigeon2(GC.PIGEON_2_CANID, Constants.ROBO_RIO_BUS_NAME);

    } 
     

    public void getVisionData(){
        /*
        var result = m_photonCamera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        SmartDashboard.putBoolean("hasTargets", hasTargets);

        if(result.hasTargets()){
             List<PhotonTrackedTarget> targets = result.getTargets();
             double Yaw = targets.get(0).getYaw();
                SmartDashboard.putNumber("Target Yaw", Yaw);

             double Pitch = targets.get(0).getPitch();
                SmartDashboard.putNumber("Target Pitch", Pitch);
        }
    */

    NetworkTable table = NetworkTableInstance.getDefault().getTable("cam");

    boolean Tv = table.getEntry("tv").getBoolean(false);
        SmartDashboard.putBoolean("Cam has targets", Tv);

    //SmartDashboard.getBoolean("Cam2 has targets", LimelightHelpers.getTV("cam2"));
    
    if(Tv == true){
        double Tx = table.getEntry("tx").getDouble(0);
            camData.put("Tx", Tx);

        double Ty = table.getEntry("ty").getDouble(0);
            camData.put("TyCam", Ty);

        double Ta = table.getEntry("ty").getDouble(0);
            camData.put("TaCam", Ta);
    }
        /*
    if(LimelightHelpers.getTV("cam2")){

        double TxCam2 = LimelightHelpers.getTX("cam2");
            cam2Data.put("TxCam2", TxCam2);

        double TyCam2 = LimelightHelpers.getTX("cam2");
            cam2Data.put("TyCam2", TyCam2);

        double TaCam2 = LimelightHelpers.getTA("cam2");
            cam2Data.put("TaCam2", TaCam2);
    }*/

    }

    


    
    @Override
    public void periodic(){



        //Yaw = camera.getAllUnreadResults().get(0);
        //SmartDashboard.putBoolean("limelight Has Targets", camera.getLatestResult().hasTargets());
        getVisionData();
       
    }

    double aimProportional(){

        double kp = .035;

        double targetingAngularVelocity = camData.get("TxCam")* kp;

        targetingAngularVelocity *= Constants.SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC;

        return targetingAngularVelocity;

    }

    double limelightRangeProportional(){    
        double kP = .1;
        double targetingForwardSpeed = camData.get("TyCam") * kP;

        targetingForwardSpeed *= Constants.SDC.MAX_ROBOT_SPEED_M_PER_SEC;
        targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
     }

    //double limelightStrafePropotional(){

      //  double kp = .01;
        //double targetingSideSpeed = 
    //}
    double limelightDistToTarget(){

        double distance = Math.sqrt(Constants.VC.APRILTAG_AREA_IN/Constants.VC.MEASURED_APRILTAG_AREA);

        return distance;
    }

    
    /*
    public Pose2d visionPoseEstimator(){
        // double Area = m_photonCamera.getLatestResult().getTargets().get(0).getArea();
        //getVisionData().targetXOffsetCam1()


    }
*/
    private void drive(){


    }

}

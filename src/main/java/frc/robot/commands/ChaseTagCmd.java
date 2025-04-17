package frc.robot.commands;

import frc.robot.subsystems.VisionTestSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ChaseTagCmd extends Command {

    private SwerveSubsystem m_swerveSubsystem;

    private VisionTestSubsystem m_visionTestSubsystem;

    private DriveToTargetCmd m_Drive = new DriveToTargetCmd(m_swerveSubsystem, m_visionTestSubsystem);
    
    private AimAtTargetCmd m_Aim;
    //private Counter m_counter;


    int counter = 0;

    public ChaseTagCmd(SwerveSubsystem swerve, VisionTestSubsystem vision){

        m_swerveSubsystem = swerve;
        m_visionTestSubsystem = vision;

        m_Drive = new DriveToTargetCmd(m_swerveSubsystem, m_visionTestSubsystem);
        m_Aim = new AimAtTargetCmd(m_swerveSubsystem, m_visionTestSubsystem);
        SmartDashboard.putNumber("counter", counter);
    }

    @Override
    public void execute(){
        
/*
        counter = counter++;
        m_swerveSubsystem.drive(m_visionTestSubsystem.targetLocation(), m_visionTestSubsystem.aimProportional(), false);
        //m_Aim.execute();
        //m_Drive.execute();
        SmartDashboard.putNumber("odometry X", m_swerveSubsystem.m_odometryPoseXEntry.getDouble(0));
        SmartDashboard.putNumber("odometry Y", m_swerveSubsystem.m_odometryPoseYEntry.getDouble(0));*/
    }  

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return true;

/*
        if(m_visionTestSubsystem.camData.get("Tv") == 0.0){
            return true;
        }

        return (m_visionTestSubsystem.aimProportional() == 0) && (m_visionTestSubsystem.getCamAreaDist() <= 48.0  +- 3);
    }*/
}}
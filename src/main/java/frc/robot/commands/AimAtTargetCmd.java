package frc.robot.commands;

import frc.robot.subsystems.VisionTestSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AimAtTargetCmd extends Command{

    private SwerveSubsystem m_SwerveSubsystem;
    private VisionTestSubsystem m_VisionTestSubsystem;

    public AimAtTargetCmd(SwerveSubsystem swerve, VisionTestSubsystem vision){
        m_SwerveSubsystem = swerve;
        m_VisionTestSubsystem = vision;
    }

    @Override
    public void execute(){
        m_SwerveSubsystem.drive(m_VisionTestSubsystem.testTranslate(), m_VisionTestSubsystem.aimProportional(), false);
    }

    @Override
    public boolean isFinished(){

        double aim = MathUtil.applyDeadband(m_VisionTestSubsystem.aimProportional(), 1);

        return aim == 0;
    }

}

package frc.robot;

import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberRealIO;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeRealIO;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterRealIO;
import frc.robot.subsystems.Transport.Transport;
import frc.robot.subsystems.Transport.TransportRealIO;

public class SubsystemManager {

    DriveTrain m_drive;
    Intake m_intake;
    Transport m_transport;
    Shooter m_shooter;
    Climber m_climber;

    

    public SubsystemManager(){
        if (Robot.isSimulation()){
            
        } else {
            m_drive = new DriveTrainRealIO();
            m_intake = new IntakeRealIO();
            m_transport = new TransportRealIO();
            m_shooter = new ShooterRealIO();
            m_climber = new ClimberRealIO();
        }
    }



}

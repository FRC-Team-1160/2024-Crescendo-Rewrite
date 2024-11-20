package frc.robot;

import java.util.function.BooleanSupplier;

import org.ejml.equation.Function;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
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

    DriveState drive_state;
    ShootState shoot_state;

    boolean note_stored;

    Pose2d tracked_note;

    PIDController drive_angle_pid;

    Pose2d robot_pose;
    SwerveDrivePoseEstimator m_pose_estimator;

    StructPublisher<Pose2d> adv_pose_pub;

    Commands commands;

    enum DriveState {
        FULL_CONTROL,
        AIMING_SPEAKER,
        AIMING_NOTE
    }

    enum ShootState {
        OFF,
        REV,
        SHOOTING
    }

    public SubsystemManager(){

        if (Robot.isSimulation()){
            
        } else {
            m_drive = new DriveTrainRealIO();
            m_intake = new IntakeRealIO();
            m_transport = new TransportRealIO();
            m_shooter = new ShooterRealIO();
            m_climber = new ClimberRealIO();
        }

        drive_state = DriveState.FULL_CONTROL;
        shoot_state = ShootState.OFF;

        note_stored = false;

        drive_angle_pid = new PIDController(0.1, 0, 0); //GET CONSTANTS
        drive_angle_pid.enableContinuousInput(-Math.PI, Math.PI);

        robot_pose = new Pose2d();
        m_pose_estimator = new SwerveDrivePoseEstimator(
            m_drive.m_kinematics, 
            m_drive.getGyroAngle(), 
            m_drive.getModulePositions(), 
            robot_pose);

        commands = new Commands();

    }

    public void setupDashboard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable adv_swerve = inst.getTable("adv_swerve");
        adv_pose_pub = adv_swerve.getStructTopic("Pose", Pose2d.struct).publish();
    }

    public void periodic(double stick_x, double stick_y, double stick_a){

        robot_pose = m_pose_estimator.update(m_drive.getGyroAngle(), m_drive.getModulePositions());

        note_stored = m_transport.note_stored;

        double drive_x = stick_x * SwerveConstants.MAX_SPEED;
        double drive_y = stick_y * SwerveConstants.MAX_SPEED;
        double drive_a = stick_a;

        switch(drive_state){
            case AIMING_SPEAKER:
                drive_angle_pid.setSetpoint(
                    Math.atan2(
                        FieldConstants.SPEAKER_Y - robot_pose.getY(), 
                        FieldConstants.SPEAKER_X - robot_pose.getX()));
                drive_a = drive_angle_pid.calculate(robot_pose.getRotation().getRadians());
                break;

            case AIMING_NOTE:
                drive_angle_pid.setSetpoint(
                    Math.atan2(
                        tracked_note.getY() - robot_pose.getY(), 
                        tracked_note.getX() - robot_pose.getX()));
                drive_a = drive_angle_pid.calculate(robot_pose.getRotation().getRadians());
                break;

            case FULL_CONTROL:

        }
        m_drive.setSwerveDrive(drive_x, drive_y, drive_a);

        switch(shoot_state){
            case SHOOTING:
            case REV:
                double dist = Math.sqrt(
                    Math.pow(FieldConstants.SPEAKER_Y - robot_pose.getY(), 2) +
                    Math.pow(FieldConstants.SPEAKER_X - robot_pose.getX(), 2));
                m_shooter.setSpeed(Math.min(3000 + 500 * dist, 5000)); //random numbers idk
                break;
            case OFF:
                m_shooter.setSpeed(0);
        }

        publishAdv();
    }

    public void publishAdv(){
        adv_pose_pub.set(robot_pose);
    }

    class Commands {
        public Command aimSpeaker(){
            return new FunctionalCommand(
                () -> drive_state = DriveState.AIMING_SPEAKER, 
                null,
                canceled -> {if (canceled) drive_state = DriveState.FULL_CONTROL;}, 
                () -> drive_state != DriveState.AIMING_SPEAKER);
        }

        public Command aimNote(){
            return new FunctionalCommand(
                () -> {if (tracked_note != null) drive_state = DriveState.AIMING_NOTE;}, 
                null,
                canceled -> {if (canceled) drive_state = DriveState.FULL_CONTROL;}, 
                () -> drive_state != DriveState.AIMING_NOTE || tracked_note == null);
        }

        public Command shoot(){
            return new FunctionalCommand(
                () -> {
                    shoot_state = ShootState.SHOOTING;
                    m_transport.transportOn();},
                null, 
                interrupted -> {
                    if (interrupted) shoot_state = ShootState.OFF;
                    m_transport.transportOff();}, 
                () -> shoot_state != ShootState.SHOOTING)
                    .raceWith(new WaitCommand(1.0));
        }

        public Command intake(){
            return new FunctionalCommand(
                () -> m_intake.setSolenoidValue(DoubleSolenoid.Value.kForward),
                null, 
                (interrupted) -> m_intake.setSolenoidValue(null), 
                () -> !note_stored);
        }
    }
}
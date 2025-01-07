package frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    public LimelightHelpers.PoseEstimate pose_estimate;

    public Vision(){

    }

    @Override
    public void periodic(){
        pose_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    }
}

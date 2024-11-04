// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public class ClimberSimIO extends Climber {

  public static ClimberRealIO m_instance;

  public DIOSim l_limit, r_limit;

  /** Creates a new ExampleSubsystem. */
  public ClimberSimIO() {

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setLeftMotor(double speed){

  }

  public void setRightMotor(double speed){

  }

  public boolean getLeftSwitch(){ //FIX (maybe)
    return false;
  }

  public boolean getRightSwitch(){ 
    return false;
  }


  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TransportConstants;

public abstract class Transport extends SubsystemBase {

  public static Transport m_instance;

  public boolean noteStored;

  public Timer color_refresh;

  public double prox;

  public static Transport getInstance(){
    if (m_instance == null){
      if (Robot.isSimulation()){
        //m_instance = new ClimberSimIO();
      } else {
        m_instance = new TransportRealIO();
      }
    }
    return m_instance;
  }

  /** Creates a new ExampleSubsystem. */

  /**
   * Example command factory method.
   *
   * @return a command
   */


  public void transportOn(){
    setWheels(TransportConstants.WHEEL_SPEED_VOLTS);
    setBelt(TransportConstants.BELT_SPEED_VOLTS);
  }

  public void transportOff(){
    setWheels(0);
    setBelt(0);
  }

  public void outtake(){
    setWheels(-TransportConstants.WHEEL_SPEED_VOLTS);
    setBelt(-TransportConstants.BELT_SPEED_VOLTS);
  }

  abstract void setWheels(double volts);

  abstract void setBelt(double volts);

  abstract double readColorProx();

  abstract boolean getSwitchPressed();

  @Override
  public void periodic() {
    if (color_refresh.hasElapsed(0.1) && prox != 0) {
      color_refresh.restart();
      readColorProx();
    }
    noteStored = (prox > TransportConstants.PROX_THRESHOLD) || getSwitchPressed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

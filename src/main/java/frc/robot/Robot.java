/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */

public class Robot extends TimedRobot {
  // Subsystems
  private DriveTrain driveTrain;
  private Lift lift;

  // Joysticks
  private Joystick leftDriverJoystick;
  private Joystick rightDriverJoystick;
  private Joystick mechJoystick;

  @Override
  public void robotInit() {
    driveTrain = new DriveTrain();
    lift = new Lift();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.periodic(leftDriverJoystick, rightDriverJoystick);
    lift.periodic(mechJoystick);
  }
}
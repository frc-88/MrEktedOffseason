/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drive;
import frc.robot.Robot;

public class AutonDriveForward extends Command {
private static final double SPEED=.2775;
private static final double TIMER=1;
private static final double ROTATIONS=49000;
private static final double REV_INCHES_CONVERT=820.94225;
private double inches=78;
private double rotations=REV_INCHES_CONVERT*inches;
  private Drive drive=Robot.m_drive;
  public AutonDriveForward() {
    requires(drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    drive.magicPosition(rotations, rotations);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.drive(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

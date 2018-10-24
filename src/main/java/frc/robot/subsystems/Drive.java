/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.Utils.*;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands

TalonSRX left,right;
VictorSPX leftFollower0, leftFollower1, leftFollower2, rightFollower0, rightFollower1, rightFollower2; 


public Drive () {
left= new TalonSRX(RobotMap.LEFT_MASTER_DRIVE_ID);
right = new TalonSRX(RobotMap.RIGHT_MASTER_DRIVE_ID);
leftFollower0 =new VictorSPX(RobotMap.LEFT_FOLLOWER00_DRIVE_ID);
leftFollower1 =new VictorSPX(RobotMap.LEFT_FOLLOWER01_DRIVE_ID);
leftFollower2 =new VictorSPX(RobotMap.LEFT_FOLLOWER02_DRIVE_ID);
rightFollower0 =new VictorSPX(RobotMap.RIGHT_FOLLOWER00_DRIVE_ID);
rightFollower1 =new VictorSPX(RobotMap.RIGHT_FOLLOWER01_DRIVE_ID);
rightFollower2 =new VictorSPX(RobotMap.RIGHT_FOLLOWER02_DRIVE_ID);

leftFollower0.follow(left);
leftFollower1.follow(left);
leftFollower2.follow(left);
rightFollower0.follow(right);
rightFollower1.follow(right);
rightFollower2.follow(right);

left.configPeakOutputForward(RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
left.configPeakOutputReverse(-RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
left.configContinuousCurrentLimit(25, RobotMap.CAN_TIMEOUT);
left.configPeakCurrentLimit(25, RobotMap.CAN_TIMEOUT);
left.configPeakCurrentDuration(0, RobotMap.CAN_TIMEOUT);

left.configOpenloopRamp(.125, RobotMap.CAN_TIMEOUT);
left.configClosedloopRamp(.125, RobotMap.CAN_TIMEOUT);

right.configPeakOutputForward(RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
right.configPeakOutputReverse(-RobotMap.DRIVE_VOLTAGE_LIMIT,RobotMap. CAN_TIMEOUT);
right.configContinuousCurrentLimit(25, RobotMap.CAN_TIMEOUT);
right.configPeakCurrentLimit(25, RobotMap.CAN_TIMEOUT);
right.configPeakCurrentDuration(0, RobotMap.CAN_TIMEOUT);

right.configOpenloopRamp(.125, RobotMap.CAN_TIMEOUT);
right.configClosedloopRamp(.125, RobotMap.CAN_TIMEOUT);

left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.CAN_TIMEOUT);
right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotMap.CAN_TIMEOUT);

/*
DataCollection.addInt(()-> left.getSelectedSensorVelocity(0));
DataCollection.addInt(()-> right.getSelectedSensorVelocity(0));

DataCollection.addDouble(()-> left.getOutputCurrent());
DataCollection.addDouble(()-> right.getOutputCurrent());
*/

}

public void drive (double leftSpd,double rightSpd) {
left.set(ControlMode.PercentOutput, -leftSpd);
right.set(ControlMode.PercentOutput, rightSpd);
}

public void pidDrive (double leftSpd,double rightSpd) {
  left.set(ControlMode.Velocity, -leftSpd);
  right.set(ControlMode.Velocity, rightSpd);
  System.out.println(leftSpd + " "+ rightSpd);
  // System.out.println(left.getSelectedSensorVelocity(0) + " " + right.getSelectedSensorVelocity(0));
  }

public void tankDrive (double leftSpd,double rightSpd) {
  //System.out.println(leftSpd + " " + rightSpd);
//leftSpd = DriveUtils.signedPow(leftSpd,RobotMap.DRIVETRAIN_SCALING_EXPONENT);
//rightSpd = DriveUtils.signedPow(rightSpd,RobotMap.DRIVETRAIN_SCALING_EXPONENT);
//drive(leftSpd, rightSpd);
double turn=(rightSpd-leftSpd)/2;
double spd=(rightSpd+leftSpd)/2;
arcadeDrive(spd,turn);
// System.out.println(turn + " " + spd);
}
public double deadbandExponential(double spd,int exp,double deadband){
  return DriveUtils.signedPow(spd, exp) * (1-deadband) + Math.signum(spd) * deadband ;
}

public void arcadeDrive(double spd, double turnSpd){


  spd = deadbandExponential(spd, RobotMap.DRIVETRAIN_SCALING_EXPONENT, RobotMap.DRIVETRAIN_SPEED_DEADBAND);
  turnSpd = deadbandExponential(turnSpd, RobotMap.DRIVETRAIN_TURN_SCALING_EXPONENT, RobotMap.DRIVETRAIN_TURN_DEADBAND);
  // System.out.println(turnSpd + " " + spd);
  

double leftSpd= spd- spd*turnSpd;
double rightSpd=spd+ spd*turnSpd;

if (spd == 0){
  leftSpd = -turnSpd;
  rightSpd = turnSpd;
  System.out.println(leftSpd + "   " + rightSpd);
}



leftSpd *= RobotMap.MAX_SPEED;
rightSpd *= RobotMap.MAX_SPEED;

pidDrive(leftSpd, rightSpd);

}
  @Override
  public void initDefaultCommand() {
   setDefaultCommand (new ArcadeDrive());
  }}

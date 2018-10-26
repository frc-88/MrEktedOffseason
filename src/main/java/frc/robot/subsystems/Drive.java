/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import javax.print.attribute.standard.RequestingUserName;

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

NetworkTableEntry leftCommandedVoltage, rightCommandedVoltage, leftSpd, rightSpd, leftCurrent, rightCurrent;


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
left.configVoltageCompSaturation(12, RobotMap.CAN_TIMEOUT);
left.enableVoltageCompensation(true);


left.configOpenloopRamp(.125, RobotMap.CAN_TIMEOUT);
left.configClosedloopRamp(.125, RobotMap.CAN_TIMEOUT);

right.configPeakOutputForward(RobotMap.DRIVE_VOLTAGE_LIMIT, RobotMap.CAN_TIMEOUT);
right.configPeakOutputReverse(-RobotMap.DRIVE_VOLTAGE_LIMIT,RobotMap. CAN_TIMEOUT);
right.configContinuousCurrentLimit(25, RobotMap.CAN_TIMEOUT);
right.configPeakCurrentLimit(25, RobotMap.CAN_TIMEOUT);
right.configPeakCurrentDuration(0, RobotMap.CAN_TIMEOUT);
right.configVoltageCompSaturation(12, RobotMap.CAN_TIMEOUT);
right.enableVoltageCompensation(true);

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

leftCommandedVoltage=Robot.dataTable.getEntry("leftCommandVoltage");
rightCommandedVoltage=Robot.dataTable.getEntry("rightCommandVoltage");
leftSpd=Robot.dataTable.getEntry("leftSpd");
rightSpd=Robot.dataTable.getEntry("rightSpd");
leftCurrent=Robot.dataTable.getEntry("leftCurrent");
rightCurrent=Robot.dataTable.getEntry("rightCurrent");

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

// System.out.println(turn + " " + spd);

spd = deadbandExponential(spd, RobotMap.DRIVETRAIN_SCALING_EXPONENT, RobotMap.DRIVETRAIN_SPEED_DEADBAND);
turn = deadbandExponential(turn, RobotMap.DRIVETRAIN_TURN_SCALING_EXPONENT, RobotMap.DRIVETRAIN_TURN_DEADBAND);
// System.out.println(turnSpd + " " + spd);


leftSpd= spd- turn;
rightSpd=spd+ turn;




leftSpd *= RobotMap.MAX_SPEED;
rightSpd *= RobotMap.MAX_SPEED;

pidDrive(leftSpd, rightSpd);
}
public double deadbandExponential(double spd,int exp,double deadband){
  return DriveUtils.signedPow(spd, exp) * (1-deadband) + Math.signum(spd) * deadband ;
}

public void arcadeDrive(double spd, double turnSpd){


  spd = deadbandExponential(spd, RobotMap.DRIVETRAIN_SCALING_EXPONENT, RobotMap.DRIVETRAIN_SPEED_DEADBAND);
  turnSpd = deadbandExponential(turnSpd, RobotMap.DRIVETRAIN_TURN_SCALING_EXPONENT, RobotMap.DRIVETRAIN_TURN_DEADBAND);
  // System.out.println(turnSpd + " " + spd);
  
turnSpd=cheesyTurn(spd,turnSpd);
double leftSpd= spd- turnSpd;
double rightSpd=spd+ turnSpd;




leftSpd *= RobotMap.MAX_SPEED;
rightSpd *= RobotMap.MAX_SPEED;

pidDrive(leftSpd, rightSpd);
}
public double cheesyTurn(double spd, double turnRate){
  if(spd==0){
    return turnRate;
  
  }
  else{
    return spd*turnRate;
  }
  
  
  }
  @Override
  public void initDefaultCommand() {
   setDefaultCommand (new TankDrive());
  }

  public void sendData(){
leftCommandedVoltage.setDouble(left.getMotorOutputVoltage());
rightCommandedVoltage.setDouble(right.getMotorOutputVoltage());
leftSpd.setDouble(left.getSelectedSensorVelocity(0));
rightSpd.setDouble(right.getSelectedSensorVelocity(0));
leftCurrent.setDouble(left.getOutputCurrent());
rightCurrent.setDouble(right.getOutputCurrent());


  }
}

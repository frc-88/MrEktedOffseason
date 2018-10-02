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
import frc.robot.*;

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
}

public void drive (double leftSpd,double rightSpd) {
left.set(ControlMode.PercentOutput, leftSpd);
right.set(ControlMode.PercentOutput, rightSpd);
}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

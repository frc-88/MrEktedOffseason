/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import frc.robot.Badlogs.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.DataCollection;
import frc.robot.commands.AutonDriveForward;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Drive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static Drive m_drive ;
  public static OI m_oi;
public static NetworkTable dataTable;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static BadLog log;
  public static long startTimeNS;


  
  
  public static String getCurrentTimeUsingDate() {
    Date date = new Date();
    return date.toString();
}



  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    startTimeNS = System.nanoTime();

    log = BadLog.init("/home/lvuser/" + getCurrentTimeUsingDate() + ".bag");

    BadLog.createValue("Lucky_Number","7");

    BadLog.createTopic("Time", "s",
          () -> ((double) (System.nanoTime() - Robot.startTimeNS)) / 1_000_000_000d);


    BadLog.createTopic("Voltage", "v",
          () -> RobotController.getBatteryVoltage());

    BadLog.createTopicStr("Mode", BadLog.UNITLESS,
          () -> DriverStation.getInstance().isDisabled() ? "Disabled"
            : DriverStation.getInstance().isAutonomous() ? "Auto"
              : DriverStation.getInstance().isOperatorControl() ? "Teleop"
                : DriverStation.getInstance().isTest() ? "Test" : "Unknown");

    dataTable=NetworkTableInstance.getDefault().getTable("dataTable");


    m_oi = new OI();
    m_drive = new Drive();
    m_chooser.addDefault("Default Auto", new AutonDriveForward());
    // chooser.addObject("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
   


    
    /*
    try{
      DataCollection.initialize();
      } catch(IOException e){
        e.printStackTrace();
      }
      */
      

      /*
      DataCollection.addBoolean(()-> RobotController.isBrownedOut());
      DataCollection.addDouble(()-> RobotController.getBatteryVoltage());
      */
      log.finishInitialization();

    }



    int countTicks = 0;
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    log.updateTopics();

    if(DriverStation.getInstance().isEnabled()){
    log.log();
    }
    else if(countTicks%50 == 0){
          log.log();
        }

    countTicks = countTicks + 1;
   /*
    try{
    DataCollection.recordData();
    } catch(IOException e){
      e.printStackTrace();
    }
    */

    m_drive.sendData();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_chooser.getSelected();
System.out.println("Autonimous Init started");
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
    //  m_autonomousCommand.start();
  //  }

  m_drive.resetEncoders();
  new AutonDriveForward().start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
   double voltage = .1;
   int count = 0;
   int direction = 1;
   final static int LOOPS_PER_SECOND = 50;

  @Override
  public void testPeriodic() {
    if(count >= LOOPS_PER_SECOND*4){
      voltage += .1;
      direction *= -1;
      count = 0;
      System.out.println(voltage);
      
    }
    else if(count>=LOOPS_PER_SECOND*3){
m_drive.drive(0,0);
    }
    else{
    double appliedVoltage = voltage*direction;
    m_drive.drive(appliedVoltage / 12, appliedVoltage / 12);
    }
    count++;
    
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import.com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.Joystick;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private static final String kDefaultAuto = "Default";
  //private static final String kCustomAuto = "My Auto";
  //private String m_autoSelected;
  //private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public TalonSRX motorfrontleft = new TalonSRX(4);
  public TalonSRX motorbackleft = new TalonSRX(3);
  public TalonSRX motorfrontright = new TalonSRX(9);
  public TalonSRX motorbackright = new TalonSRX(8);
  Joystick stick = new Joystick(1);
  
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    motorfrontleft.set(ControlMode.PercentOutput, 0.0);
    motorfrontright.set(ControlMode.PercentOutput, 0.0);
    motorbackleft.set(ControlMode.PercentOutput, 0.0);
    motorbackright.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   * /
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   * /
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
     autoSelected = SmartDashboard.getString("Auto Selector",
     defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    /*switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break; 

      
    }*/
    drive(0.0, 0.5, 0.0, 1.0);
  }
 



  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    drive(stick.getX(), stick.getY(), stick.getZ(), (stick.getThrottle()+1.0)/2.0);
  }

  /**
   * This function is called periodically during test mode.
   * /
  @Override
  public void testPeriodic() {
  }*/
  
  
  public void drive(double x, double y, double z, double throttle) {
    double maximum;
    double speedfrontleft = (x + y + z)*throttle;
    double speedbackleft = (-x + y + z)*throttle;
    double speedfrontright = (x - y + z)*throttle;
    double speedbackright = (-x - y + z)*throttle;

    maximum = Math.max( Math.max(Math.abs(speedfrontleft), Math.abs(speedfrontright)),
                        Math.max(Math.abs(speedbackleft), Math.abs(speedbackright)) );

    if (maximum > 1.0) {
      speedfrontleft  = speedfrontleft   / maximum;
      speedfrontright = speedfrontright /  maximum;
      speedbackleft   = speedbackleft   / maximum;
      speedbackright  = speedbackright / maximum;
    }

    motorfrontleft.set(ControlMode.PercentOutput, speedfrontleft);
    motorfrontright.set(ControlMode.PercentOutput, speedfrontright);
    motorbackleft.set(ControlMode.PercentOutput, speedbackleft);
    motorbackright.set(ControlMode.PercentOutput, speedbackright);

    System.out.println("speedbackright=" + speedbackright);
    System.out.println(" speedbackleft=" + speedbackleft);
    System.out.println(" speedfrontleft=" + speedfrontleft);
    System.out.println(" speedfrontright=" + speedfrontright);
    System.out.println(" x=" + x);
    System.out.println(" y=" + y);
    System.out.println(" z=" + z);
    System.out.println(" throttle=" + throttle);
  }
}

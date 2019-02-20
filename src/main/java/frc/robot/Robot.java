/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.*;
//import edu.wpi.first.wpilibj.Timer;

//import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.cscore.CameraServerJNI;
//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;





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
  public CANSparkMax motorfrontleft = new CANSparkMax(21, MotorType.kBrushless);
  public CANSparkMax motorfrontright = new CANSparkMax(23, MotorType.kBrushless);
  public CANSparkMax motorbackleft = new CANSparkMax(22, MotorType.kBrushless);
  public CANSparkMax motorbackright = new CANSparkMax(20, MotorType.kBrushless);
  Joystick stick = new Joystick(1);
  SmartDashboard shuffleboard;
  public int sendval;
  SerialPort arduino = new SerialPort(57600,SerialPort.Port.kUSB);
  public Boolean seeking;
  public Boolean left;
  public Boolean right;
  public Boolean found;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  public CANEncoder frontleftencoder;
  public CANEncoder frontrightencoder;
  public CANEncoder backleftencoder;
  public CANEncoder backrightencoder;
  public CANPIDController frontleftpidController;
  public CANPIDController backleftpidController;
  public CANPIDController frontrightpidController;
  public CANPIDController backrightpidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public int optablestaybutton=0;
  public int optabledrivebutton=0;
  public int optablescorebutton=0;
  Compressor compressor = new Compressor(32);
  public Solenoid grip = new Solenoid(32, 2);
  public Solenoid release = new Solenoid(32, 3);
  public Solenoid push = new Solenoid(32, 0);
  public Solenoid pull = new Solenoid(32, 1);
  public double inchPerTick = 0.02025828;
  public double startingPositionDistanceForwardDrive;
  public int autonState;
  public double targetDistance;
  public double encoderConvertInch=((Math.PI*6*13)/70);
  public long time;
  AHRS navX;
  
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    motorfrontleft.set( 0.0);
    motorfrontright.set(0.0);
    motorbackleft.set(0.0);
    motorbackright.set(0.0);

    //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    // camera.setResolution(640, 480);

    arduino.reset();
    cancelSeek();
    SmartDashboard.putBoolean("seeking", false);
    SmartDashboard.putBoolean("left", false);
    SmartDashboard.putBoolean("right", false);
    frontleftencoder = motorfrontleft.getEncoder();
    frontrightencoder = motorfrontright.getEncoder();
    backleftencoder = motorbackleft.getEncoder();
    backrightencoder = motorbackright.getEncoder();
    
    table.getEntry("ledMode").setNumber(1);
    table.getEntry("camMode").setNumber(1);

    frontleftpidController = motorfrontleft.getPIDController();
    frontrightpidController = motorfrontright.getPIDController();
    backleftpidController = motorbackleft.getPIDController();
    backrightpidController = motorbackright.getPIDController();
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    frontleftpidController.setP(kP);
    frontleftpidController.setI(kI);
    frontleftpidController.setD(kD);
    frontleftpidController.setIZone(kIz);
    frontleftpidController.setFF(kFF);
    frontleftpidController.setOutputRange(kMinOutput, kMaxOutput);
    frontrightpidController.setP(kP);
    frontrightpidController.setI(kI);
    frontrightpidController.setD(kD);
    frontrightpidController.setIZone(kIz);
    frontrightpidController.setFF(kFF);
    frontrightpidController.setOutputRange(kMinOutput, kMaxOutput);
    backleftpidController.setP(kP);
    backleftpidController.setI(kI);
    backleftpidController.setD(kD);
    backleftpidController.setIZone(kIz);
    backleftpidController.setFF(kFF);
    backleftpidController.setOutputRange(kMinOutput, kMaxOutput);
    backrightpidController.setP(kP);
    backrightpidController.setI(kI);
    backrightpidController.setD(kD);
    backrightpidController.setIZone(kIz);
    backrightpidController.setFF(kFF);
    backrightpidController.setOutputRange(kMinOutput, kMaxOutput);
    compressor.setClosedLoopControl(true);
    push.set(false);
    pull.set(false);
    grip.set(false);
    release.set(false);
    startingPositionDistanceForwardDrive=0;
    autonState=0;
    navX=new AHRS(SPI.Port.kMXP);
    navX.reset();
  }

  /*
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    byte buf[] = arduino.read(1);
    sendval=buf[0];
    SmartDashboard.putNumber("Line", sendval);
  }

  /*
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well. */
   
  @Override
  public void autonomousInit() {
    /*m_autoSelected = m_chooser.getSelected();
     autoSelected = SmartDashboard.getString("Auto Selector",
     defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected); */
    //c.start();
    arduino.reset();
    autonState = 0;
    gripHatch();
    pullHatch();
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
   autonscoreleftfromcenter();
  }
 


  /**
   * This function is called periodically during operator control.
   */
  @Override 
  public void teleopInit(){
    compressor.start();
    arduino.reset();
    System.out.println("teleopInit is running");
  }

  @Override
  public void teleopPeriodic() {
    /*pull.set(false);
    push.set(false);*/
   /* NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    SmartDashboard.putNumber("Joystick X value", stick.getX());
    SmartDashboard.putNumber("Joystick throttle value", (stick.getThrottle()+1)/2);*/
    
    SmartDashboard.putNumber("Bytes", arduino.getBytesReceived());
    if (stick.getRawButtonPressed(10)) {
      cancelSeek();
    } else if (stick.getRawButtonPressed(5)) {
      startSeekLeft();
    } else if (stick.getRawButtonPressed(6)) {
      startSeekRight();
    }
    double throttle=stick.getThrottle();
    double x=joystickScaleMath(stick.getX());
    double y=joystickScaleMath(stick.getY());
    SmartDashboard.putNumber("yInput", stick.getY());
    SmartDashboard.putNumber("scaledJoy", y);
    double z=joystickScaleMath(stick.getZ());
    if (seeking) findline();
    else drive(x, y, z, (throttle+1.0)/2.0);
 

    SmartDashboard.putBoolean("seeking", seeking);
    SmartDashboard.putBoolean("left", left);
    SmartDashboard.putBoolean("right", right); 
    /*double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);*/
    if (stick.getRawButtonPressed(2)) {
      gripHatch();
    }
    if (stick.getRawButtonPressed(1)) {
      releaseHatch();
    }
    if (stick.getRawButtonPressed(3)) {
      pushHatch();
    }
    if (stick.getRawButtonPressed(4)) {
      pullHatch();
    }
    // System.out.println(frontleftencoder.getPosition());
    SmartDashboard.putNumber("x", stick.getX());
    SmartDashboard.putNumber("y", stick.getY());
    SmartDashboard.putNumber("z", stick.getZ());
    SmartDashboard.putNumber("throttle", stick.getThrottle());

  }

  /**
   * This function is called periodically during test mode.
   * /
  @Override
  public void testPeriodic() {
  }*/
  
  
  public void drive(double x, double y, double z, double throttle) {
    double maximum;
    double speedfrontleft = (x - y + 0.75*z)*throttle;
    double speedbackleft = (-x - y + 0.75*z)*throttle;
    double speedfrontright = (x + y + 0.75*z)*throttle;
    double speedbackright = (-x + y + 0.75*z)*throttle;
    

    maximum = Math.max( Math.max(Math.abs(speedfrontleft), Math.abs(speedfrontright)),
                        Math.max(Math.abs(speedbackleft), Math.abs(speedbackright)) );

    if (maximum > 1.0) {
      speedfrontleft  = speedfrontleft   / maximum;
      speedfrontright = speedfrontright /  maximum;
      speedbackleft   = speedbackleft   / maximum;
      speedbackright  = speedbackright / maximum;
    }
    //negative because not direct driving
    //double maxrpm=-5676;
    double maxrpm=-2838;
    frontleftpidController.setReference(speedfrontleft*maxrpm, ControlType.kVelocity);
    backleftpidController.setReference(speedbackleft*maxrpm, ControlType.kVelocity);
    backrightpidController.setReference(speedbackright*maxrpm, ControlType.kVelocity);
    frontrightpidController.setReference(speedfrontright*maxrpm, ControlType.kVelocity);
    

    /*System.out.println("speedbackright=" + speedbackright);
    System.out.println(" speedbackleft=" + speedbackleft);
    System.out.println(" speedfrontleft=" + speedfrontleft);
    System.out.println(" speedfrontright=" + speedfrontright);
    System.out.println(" x=" + x);
    System.out.println(" y=" + y);
    System.out.println(" z=" + z);
    System.out.println(" throttle=" + throttle);*/

    /*System.out.println(motorbackright.getSelectedSensorPosition());
    System.out.println(motorbackleft.getSelectedSensorPosition());
    System.out.println(motorfrontright.getSelectedSensorPosition());
    System.out.println(motorfrontleft.getSelectedSensorPosition());*/
  }
  public boolean findline() {
    int goal=9;
    
    if ((sendval >=0) && (sendval < 8)) {
      right = true;
      left = false;
      found = true;
    } else if ((sendval <=15) && (sendval >= 8)) {
      left = true;
      right = false;
      found = true;
    } else if (found == true) {
      cancelSeek();
    }
/*
    if (right) {
      goal = 6;
    } else {
      goal = 9;
    }
    */

    int distance = Math.abs(sendval-goal);
    double xspeed = 0.15 + (double)distance/90.0; // max throttle is 0.2
    pullHatch();
   
    if (left) {
      xspeed=-xspeed;
    }
    double throttle=stick.getThrottle();
    double y=joystickScaleMath(stick.getY());
    drive(xspeed, y, 0.0, (throttle+1.0)/2.0);

    if (distance <= 1) {
      //cancelSeek();
      //drive(0.0, 0.0, 0.0, 0.0);
      return true;
    } else {
      return false;
    }
  } 

  public void flipdrive(double x, double y, double z, double throttle) {
    double maximum;
    double speedfrontleft = (x - y + z)*throttle;
    double speedbackleft = (-x - y + z)*throttle;
    double speedfrontright = (x + y + z)*throttle;
    double speedbackright = (-x + y + z)*throttle;

    maximum = Math.max( Math.max(Math.abs(speedfrontleft), Math.abs(speedfrontright)),
                        Math.max(Math.abs(speedbackleft), Math.abs(speedbackright)) );

    if (maximum > 1.0) {
      speedfrontleft  = speedfrontleft   / maximum;
      speedfrontright = speedfrontright /  maximum;
      speedbackleft   = speedbackleft   / maximum;
      speedbackright  = speedbackright / maximum;
    }

    motorfrontleft.set(-speedfrontleft);
    motorfrontright.set(-speedfrontright);
    motorbackleft.set(-speedbackleft);
    motorbackright.set(-speedbackright);
  }
  public void gripHatch() {
    grip.set(true);
    release.set(false);
    cancelSeek();
  }
  public void releaseHatch() {
    release.set(true);
    grip.set(false);
    cancelSeek();
  }
  public void pushHatch() {
    push.set(true);
    pull.set(false);
    cancelSeek();
  }
  public void pullHatch() {
    pull.set(true);
    push.set(false);
  }


  public boolean distanceforwarddrive(double distance) {
    double currentDistance=frontleftencoder.getPosition()*encoderConvertInch;
    System.out.println("current distance "+currentDistance);
    System.out.println("current position "+frontleftencoder.getPosition());
    if(distance<currentDistance) {
      drive (0.0, -0.5, 0.0, 0.3);
      return false;
      /*distance=distance-((frontleftencoder.getPosition()-lastposition))*inchPerTick; 
      lastposition=frontleftencoder.getPosition();*/
    }else{
      drive (0.0, 0.0, 0.0, 0.0);
      return true;
    }
  }
  

  public void autonscoreleftfromcenter() {  switch(autonState){
    case 0:
      System.out.println("In Auton State 0!");
      targetDistance=frontleftencoder.getPosition()*encoderConvertInch-135;
      autonState = 1;
      break;
    case 1:
      System.out.println("In Auton State 1!");
      if (distanceforwarddrive(targetDistance))
      {
        autonState = 2;
        startSeekLeft();
      }
      break;
    case 2:
      System.out.println("In Auton State 2!");
      if (findline()) {
        autonState = 3;
        time=System.currentTimeMillis();
        drive(0.0, 0.0, 0.0, 0.0);
      }
      break;
    case 3:
    System.out.println("In Auton State 3!");
      pushHatch();
      if (System.currentTimeMillis()-time>2000)
      {
        autonState=4;
        time=System.currentTimeMillis();
      }
      
      break;
    case 4:
    System.out.println("In Auton State 4!");
      releaseHatch();
     if (System.currentTimeMillis()-time>2000)
     {
       autonState=5;
       time=System.currentTimeMillis();
     }
     
      break;
    case 5:
    System.out.println("In Auton State 5!");
      pullHatch();
      break;

  }
  }
  public void autonscorerightfromcenter() {
    switch(autonState){
      case 0:
        System.out.println("In Auton State 0!");
        targetDistance=frontleftencoder.getPosition()*encoderConvertInch-135;
        autonState = 1;
        break;
      case 1:
        System.out.println("In Auton State 1!");
        if (distanceforwarddrive(targetDistance))
        {autonState = 2;}
        break;
      case 2:
        startSeekLeft();
        System.out.println("In Auton State 2!");
        if (findline()) {
          autonState = 3;
          time=System.currentTimeMillis();
        }
        break;
      case 3:
      System.out.println("In Auton State 3!");
        pushHatch();
        if (System.currentTimeMillis()-time>2000)
        {
          autonState=4;
          time=System.currentTimeMillis();
        }
        
        break;
      case 4:
      System.out.println("In Auton State 4!");
        releaseHatch();
       if (System.currentTimeMillis()-time>2000)
       {
         autonState=5;
         time=System.currentTimeMillis();
       }
       
        break;
      case 5:
      System.out.println("In Auton State 5!");
        pullHatch();
        break;
  
    }
    }

  public void autondriveforward() {
    switch(autonState){
      case 0:
        System.out.println("In Auton State 0!");
        targetDistance=frontleftencoder.getPosition()*encoderConvertInch-50;
        autonState = 1;
        break;
      case 1:
        distanceforwarddrive(targetDistance);
        break;
    }
    
  }
  /*public void limelightseek() {
    double v = tv.getDouble(0.0);
    if (v=0.0) {}
    } */

  public void autonscorestartright() {
    switch(autonState) {
      case 0:
      System.out.println("In State0!");
      targetDistance=frontleftencoder.getPosition()*encoderConvertInch-175.55;
      autonState=1;
      break;
      case 1:
      System.out.println("In Auton State 1!");
        if (distanceforwarddrive(targetDistance))
        {autonState = 2;}
      break;
      case 2:
      System.out.println("In Auton State 2!");
      drive(0, 0, -1.0, 0.15);
      if (navX.getAngle()<=-90) {autonState=3;}
      break;
      case 3:
      System.out.println("In Auton State 3!");
      targetDistance=frontleftencoder.getPosition()*encoderConvertInch-18.065;
      autonState=4;
      break;
      case 4:
      System.out.println("In Auton State 4!");
      if (distanceforwarddrive(targetDistance)) {autonState=5;}
      break;
      case 5:
      System.out.println("In Auton State 5!");
      startSeekRight();
      if (findline()) {autonState=6;
      time=System.currentTimeMillis();}
      break;
      case 6:
      System.out.println("In Auton State 6!");
      pushHatch();
      if (System.currentTimeMillis()-time>=2000) {
        autonState=7;
        time=System.currentTimeMillis();
      }
      break;
      case 7:
      System.out.println("In Auton State 7!");
      releaseHatch();
      if (System.currentTimeMillis()-time>=2000) {
        autonState=8;
        time=System.currentTimeMillis();
      }
      break;
      case 8:
      pullHatch();
      break;

    }

  }
  public double joystickScaleMath(double joyInput) {
    double joyMag;
  
    // Constants for Calculation
    double minJoy=0.02;
    double transJoy=0.5;
    double minOut=0;
    double transOut=0.25;
    double yIntSlow;
    double yIntFast;

    // Return Value
    double scaledJoy;

    // Find magnitude of scaled joy 
    joyMag=Math.abs(joyInput);
    if (joyMag<=minJoy) {
      scaledJoy= minOut; 
    }
    else if (joyMag<=transJoy) {
      double slope = (transOut-minOut)/(transJoy-minJoy);
      yIntSlow=transOut-transJoy*slope;
      scaledJoy = slope*joyMag+yIntSlow;
    }
    else {
      double slope = ((1-transOut)/(1-transJoy));
      yIntFast=transOut-transJoy*slope;
      scaledJoy = slope*(joyMag)+yIntFast;
    }

    // Correct direction of scaled joy
    if (joyInput<0) {
      scaledJoy=-1*scaledJoy;
    }
    

    return scaledJoy;
  }
public void score() {
  releaseHatch();
  pushHatch();
  pullHatch();
  gripHatch();
  targetDistance=40;
  distanceforwarddrive(targetDistance);

}
public void intake() {
  releaseHatch();
  pushHatch();
  gripHatch();
  pullHatch();
  targetDistance=40;
  distanceforwarddrive(targetDistance);
}
public void autonscorestartleft() {
  switch(autonState) {
    case 0:
    System.out.println("In State0!");
    targetDistance=frontleftencoder.getPosition()*encoderConvertInch-175.55;
    autonState=1;
    break;
    case 1:
    System.out.println("In Auton State 1!");
      if (distanceforwarddrive(targetDistance))
      {autonState = 2;}
    break;
    case 2:
    System.out.println("In Auton State 2!");
    drive(0, 0, 1.0, 0.15);
    if (navX.getAngle()>=90) {autonState=3;}
    break;
    case 3:
    System.out.println("In Auton State 3!");
    targetDistance=frontleftencoder.getPosition()*encoderConvertInch-18.065;
    autonState=4;
    break;
    case 4:
    System.out.println("In Auton State 4!");
    if (distanceforwarddrive(targetDistance)) {autonState=5;}
    break;
    case 5:
    System.out.println("In Auton State 5!");
    startSeekLeft();
    if (findline()) {autonState=6;
    time=System.currentTimeMillis();}
    break;
    case 6:
    System.out.println("In Auton State 6!");
    pushHatch();
    if (System.currentTimeMillis()-time>=2000) {
      autonState=7;
      time=System.currentTimeMillis();
    }
    break;
    case 7:
    System.out.println("In Auton State 7!");
    releaseHatch();
    if (System.currentTimeMillis()-time>=2000) {
      autonState=8;
      time=System.currentTimeMillis();
    }
    break;
    case 8:
    pullHatch();
    break;

  }

}
public void cancelSeek() {
  seeking=false;
  right=false;
  left=false;
  found=false;
}

public void startSeekLeft() {
  seeking=true;
  right=false;
  left=true;
  found=false;
}

public void startSeekRight() {
  seeking=true;
  right=true;
  left=false;
  found=false;
}

public boolean findlineBackup() {
  int goal;
  if (left) {goal=9;}
  else {goal=6;}
  int distance = Math.abs(sendval-goal);
    double throttle = 0.15 + (double)distance/90; // max throttle is 0.3
    pullHatch();
   
    if (left) 
    {
      drive(-0.3, 0, 0, throttle);
    }else if (right)
    { 
      drive(0.3, 0, 0, throttle);
    }

    if (distance <= 1) {
      cancelSeek();
      drive(0, 0, 0, 0);
      return true;
    }
    else {return false;}
  }
}

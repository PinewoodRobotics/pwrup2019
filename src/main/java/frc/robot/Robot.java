/**---------------------------------------------------------------------------*
 * Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        *
 * Open Source Software - may be modified and shared by FRC teams. The code   *
 * must be accompanied by the FIRST BSD license file in the root directory of *
 * the project.                                                               *
 *---------------------------------------------------------------------------**/

package frc.robot;

/** WPI Core Libraries **/
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI;

/** WPI Dashboard Libraries **/
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.shuffleboard.*;

/** WPI Video Libraries **/
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.cscore.CameraServerJNI;
//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.cscore.VideoSource.ConnectionStrategy;

/** REV Robotics Libraries (SparkMax) **/
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

/** Cross The Road Electronics Libraries (TalonSRX) **/
//import com.ctre.phoenix.motorcontrol.DemandType;

// Kauai Labs libraries (NavX)
import com.kauailabs.navx.frc.AHRS;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    /** Objects representing hardware attached to the RoboRIO **/
    CANSparkMax motorfrontleft = new CANSparkMax(21, MotorType.kBrushless);
    CANSparkMax motorfrontright = new CANSparkMax(23, MotorType.kBrushless);
    CANSparkMax motorbackleft = new CANSparkMax(22, MotorType.kBrushless);
    CANSparkMax motorbackright = new CANSparkMax(20, MotorType.kBrushless);
    Compressor compressor = new Compressor(32);
    Solenoid push = new Solenoid(32, 0);        // arm out
    Solenoid pull = new Solenoid(32, 1);        // arm in
    Solenoid grip = new Solenoid(32, 2);        // scissor in
    Solenoid release = new Solenoid(32, 3);     // scissor out
    SerialPort arduino = new SerialPort(57600,SerialPort.Port.kUSB);    // line tracker
    AHRS navX = new AHRS(SPI.Port.kMXP);        // Used for auton navigation

    /** Objects & Variables representing the Drive Station **/
    Joystick stick = new Joystick(1);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    //NetworkTableEntry tx = table.getEntry("tx");
    //NetworkTableEntry tv = table.getEntry("tv");

    /** Objects, Variables, and Constants used for the drive system **/
    CANEncoder frontleftencoder;
    CANEncoder frontrightencoder;
    CANEncoder backleftencoder;
    CANEncoder backrightencoder;
    CANPIDController frontleftpidController;
    CANPIDController backleftpidController;
    CANPIDController frontrightpidController;
    CANPIDController backrightpidController;
    final double scaleRotation = 0.75; // Robot rotates too
    final double kP = 5e-5;
    final double kI = 1e-6;
    final double kD = 0.0;
    final double kIz = 0.0;
    final double kFF = 0.0;
    final double kMaxOutput = 1.0;
    final double kMinOutput = -1.0;
    final double maxRPM = -2838.0;  // Negative because gearing reverses rotation
    final double encoderConvertInch=((Math.PI*6.0*13.0)/70.0);
    //final double inchPerTick = 0.02025828;

    /** Variables used for line tracking **/
    int sendval;
    Boolean seeking;
    Boolean left;
    Boolean right;
    Boolean found;

    /** Objects & Variables used for autonomous **/
    double startingPositionDistanceForwardDrive;
    int autonState;
    double targetDistance;
    long time;
    //static final String kDefaultAuto = "Default";
    //static final String kCustomAuto = "My Auto";
    //String m_autoSelected;
    //final SendableChooser<String> m_chooser = new SendableChooser<>();
    //int optablestaybutton=0;
    //int optabledrivebutton=0;
    //int optablescorebutton=0;

    /**
     * This function runs once when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Setup Drive Base
        motorfrontleft.set(0.0);
        motorfrontright.set(0.0);
        motorbackleft.set(0.0);
        motorbackright.set(0.0);

        frontleftencoder = motorfrontleft.getEncoder();
        frontrightencoder = motorfrontright.getEncoder();
        backleftencoder = motorbackleft.getEncoder();
        backrightencoder = motorbackright.getEncoder();

        frontleftpidController = motorfrontleft.getPIDController();
        frontrightpidController = motorfrontright.getPIDController();
        backleftpidController = motorbackleft.getPIDController();
        backrightpidController = motorbackright.getPIDController();

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

        // Initialize pneumatics
        compressor.setClosedLoopControl(true);
        push.set(false);
        pull.set(false);
        grip.set(false);
        release.set(false);

        // Setup camera streaming
        //UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        //camera.setResolution(640, 480);
        table.getEntry("ledMode").setNumber(1);
        table.getEntry("camMode").setNumber(1);

        // Initialize autonomous & line seeking logic
        navX.reset();
        arduino.reset();    // clear all unprocessed line tracker reports
        startingPositionDistanceForwardDrive = 0.0;
        autonState = 0;
        cancelSeek();

        // Autonomous Selecting Logic
        //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        //m_chooser.addOption("My Auto", kCustomAuto);

        // Initialize values on dashboard
        SmartDashboard.putBoolean("seeking", false);
        SmartDashboard.putBoolean("left", false);
        SmartDashboard.putBoolean("right", false);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     **/
    @Override
    public void robotPeriodic() {
        byte buf[] = arduino.read(1);
        sendval=buf[0];
        SmartDashboard.putNumber("Line", sendval);
        SmartDashboard.putNumber("Bytes", arduino.getBytesReceived());
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
     */

    @Override
    public void autonomousInit() {
        /*m_autoSelected = m_chooser.getSelected();
         autoSelected = SmartDashboard.getString("Auto Selector",
         defaultAuto);
         System.out.println("Auto selected: " + m_autoSelected); */
        //c.start();
        compressor.start();
        arduino.reset();    // clear all unprocessed line tracker reports
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
        arduino.reset();    // clear all unprocessed line tracker reports
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


    /**
     * drive(x,y,z,t) commands the motor controllers to move based on
     * the provided axis inputs.
     * x controls side-to-side movement (crabbing).
     * y controls front-to back movement.
     * z controls rotation
     * throttle scales the x,y,z axis inputs
     * x,y,z values range from -1.0 to +1.0
     * throttle ranges from 0.0 (stopped) to 1.0 (full speed).
     */

    public void drive(double x, double y, double z, double throttle) {

        // Scale the three input axis by throttle, and reduce rotation.
        x = x * throttle;
        y = y * throttle;
        z = z * throttle * scaleRotation;

        // Determine the maximum magnitute of all axes
        double maximum = Math.abs(x) + Math.abs(y) + Math.abs(z);

        // Compute the RPM of each motor (Mecanum Drive)
        double speedfrontleft  = ( x - y + z) * maxRPM;
        double speedfrontright = ( x + y + z) * maxRPM;
        double speedbackleft   = (-x - y + z) * maxRPM;
        double speedbackright  = (-x + y + z) * maxRPM;

        // If any wheels are saturated, scale them all down
        if (maximum > 1.0) {
            speedfrontleft  = speedfrontleft  / maximum;
            speedfrontright = speedfrontright / maximum;
            speedbackleft   = speedbackleft   / maximum;
            speedbackright  = speedbackright  / maximum;
        }

        // Command the PID controllers to the calculated speed
        frontleftpidController.setReference(speedfrontleft, ControlType.kVelocity);
        backleftpidController.setReference(speedbackleft, ControlType.kVelocity);
        backrightpidController.setReference(speedbackright, ControlType.kVelocity);
        frontrightpidController.setReference(speedfrontright, ControlType.kVelocity);
    }

    public boolean findline() {
        final int goal = 9;

        pullHatch(); // We never want to seek with the arm extended

        if ((sendval >= 0) && (sendval <= 15)) {
            // Line tracker is seeing tape now
            found = true;
            if (sendval < goal) {
                // Need to move robot right to center tape
                right = true;
                left = false;
            } else if (sendval > goal) {
                // Need to move robot left to center tape
                left = true;
                right = false;
            }
        } else if (found == true) {
            // We lost the line - not sure which way to go, so cancel
            cancelSeek();
            // XXX - could remember which side last saw the line and seek that way instead of cancelling
        }

        // Compute seek speed based on how far off center we are
        int distance = Math.abs(sendval-goal);
        double xspeed = 0.15 + (double)distance/90.0; // max throttle is 0.2

        if (left)
            xspeed=-xspeed;

        // Allow the driver to operate the Y axis while line tracker handles X
        double throttle=stick.getThrottle();
        double y=joystickScaleMath(stick.getY());
        drive(xspeed, y, 0.0, (throttle+1.0)/2.0);  // XXX - throttle effects our seeking speed but should not!

        // return true if we are centered over the tape
        if (distance <= 1)
            return true;
        else
            return false;
    }

    /**
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
    } **/

    // Closes the Scissor to capture a hatch
    // Also cancels any seek in progress
    public void gripHatch() {
        grip.set(true);
        release.set(false);
        cancelSeek();
    }

    // Opens the Scissor to place a hatch
    // Also cancels any seek in progress
    public void releaseHatch() {
        release.set(true);
        grip.set(false);
        cancelSeek();
    }
    // Extends the arm to place a hatch
    // Also cancels any seek in progress
    public void pushHatch() {
        push.set(true);
        pull.set(false);
        cancelSeek();
    }
    // Retracts the arm for driving
    // Does NOT cancel seek, since this is called by findline()
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
        } else {
            drive (0.0, 0.0, 0.0, 0.0);
            return true;
        }
    }


    public void autonscoreleftfromcenter() {
        switch(autonState){
            case 0:
                System.out.println("In Auton State 0!");
                targetDistance=frontleftencoder.getPosition()*encoderConvertInch-135;
                autonState = 1;
                break;
            case 1:
                System.out.println("In Auton State 1!");
                if (distanceforwarddrive(targetDistance)) {
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
                if (System.currentTimeMillis()-time>2000) {
                    autonState=4;
                    time=System.currentTimeMillis();
                }

                break;
            case 4:
                System.out.println("In Auton State 4!");
                releaseHatch();
                if (System.currentTimeMillis()-time>2000) {
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
                if (System.currentTimeMillis()-time>2000) {
                    autonState=4;
                    time=System.currentTimeMillis();
                }

                break;
            case 4:
                System.out.println("In Auton State 4!");
                releaseHatch();
                if (System.currentTimeMillis()-time>2000) {
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

    /**
    public void limelightseek() {
        double v = tv.getDouble(0.0);
        if (v=0.0) {}
    }
     **/

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

    /**
     * joystickScaleMath() takes a joystick axis value and applies a function
     * that provides a deadband, as well as fast and slow speed ranges.
     **/
    public double joystickScaleMath(double joyInput) {
        // Constants for input values
        final double minJoy   = 0.02;   // deadband below this value
        final double transJoy = 0.50;   // slow/fast transition point

        // Constants for output values
        final double minOut   = 0.00;   // output when at minJoy
        final double transOut = 0.25;   // output at slow/fast transition

        // Return Value
        double scaledJoy = 0.0;

        // Find magnitude of scaled joy
        double joyMag = Math.abs(joyInput);

        // Enforce the deadband
        if (joyMag <= minJoy) {
            scaledJoy = minOut;
        } else if (joyMag <= transJoy) {
            // Slow speed range
            double slope = ( (transOut-minOut) / (transJoy-minJoy) );
            double intercept = transOut - (transJoy * slope);
            scaledJoy = intercept + (joyMag * slope);
        } else {
            // Fast speed range
            double slope = ( (1.0-transOut) / (1.0-transJoy) );
            double intercept = transOut - (transJoy * slope);
            scaledJoy = intercept + (joyMag * slope);
        }

        // Correct direction of scaled joy
        if (joyInput < 0.0)
            scaledJoy = -scaledJoy;

        return scaledJoy;
    }

    public void score() {
        releaseHatch();
        pushHatch();
        pullHatch();
        gripHatch();
        targetDistance=40.0;
        distanceforwarddrive(targetDistance);

    }

    public void intake() {
        releaseHatch();
        pushHatch();
        gripHatch();
        pullHatch();
        targetDistance=40.0;
        distanceforwarddrive(targetDistance);
    }

    public void autonscorestartleft() {
        switch(autonState) {
            case 0:
                System.out.println("In State0!");
                targetDistance = frontleftencoder.getPosition() * encoderConvertInch - 175.55;
                autonState = 1;
                break;
            case 1:
                System.out.println("In Auton State 1!");
                if (distanceforwarddrive(targetDistance))
                    autonState = 2;
                break;
            case 2:
                System.out.println("In Auton State 2!");
                drive(0, 0, 1.0, 0.15);
                if (navX.getAngle()>=90)
                    autonState = 3;
                break;
            case 3:
                System.out.println("In Auton State 3!");
                targetDistance = frontleftencoder.getPosition() * encoderConvertInch - 18.065;
                autonState = 4;
                break;
            case 4:
                System.out.println("In Auton State 4!");
                if (distanceforwarddrive(targetDistance))
                    autonState = 5;
                break;
            case 5:
                System.out.println("In Auton State 5!");
                startSeekLeft();
                if (findline()) {
                    autonState = 6;
                    time = System.currentTimeMillis();
                }
                break;
            case 6:
                System.out.println("In Auton State 6!");
                pushHatch();
                if (System.currentTimeMillis() - time >= 2000) {
                    autonState = 7;
                    time=System.currentTimeMillis();
                }
                break;
            case 7:
                System.out.println("In Auton State 7!");
                releaseHatch();
                if (System.currentTimeMillis() - time >= 2000) {
                    autonState = 8;
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

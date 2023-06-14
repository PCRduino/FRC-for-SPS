// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// odom/justinvil imports:
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  // default constructors
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);     // PWM pins
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);    // PWM pins
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  // Justinvil-Odom Constructors:
  DigitalOutput odomBuzzer = new DigitalOutput(6);        // DIO pin 6 
  DigitalOutput odomRedLED = new DigitalOutput(7);       // RGB led connected to DIO pins 7, 8, 9
  DigitalOutput odomGreenLED = new DigitalOutput(8);       // RGB led connected to DIO pins 7, 8, 9
  DigitalOutput odomBlueLED = new DigitalOutput(9);       // RGB led connected to DIO pins 7, 8, 9
  DigitalInput odomButton = new DigitalInput(0);        // DIO pin 0  
  AnalogInput odomIRRanger = new AnalogInput(0);           // Analog pin 0  
  Servo odomSRCServo = new Servo(8);                      // continuous 360-degree SpringRC on PWM pin 8
  Servo odomHitecServo = new Servo(9);                      // 180-degree Hitec on PWM pin 8
  XboxController odomXbox = new XboxController(0);        // plug in Xbox controller to USB port on laptop.  See Driver Station >> USB tab for the port number (probably 0)

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    System.out.println("hi 4");

    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }

    System.out.println("hi 5");

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    System.out.println("hi 2");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    System.out.println("hi 3");

    // timer functions:
    System.out.println("Robot Running Time = " + Timer.getFPGATimestamp() );
    Timer.delay(0.5);      // delay for 0.5 seconds
    System.out.println("Match Time = " + Timer.getMatchTime() );

    // RGB LED functions brute force
    /*
    double myLEDTime = 0.5;
    odomRedLED.set(true);
    odomGreenLED.set(false);
    odomBlueLED.set(false);
    Timer.delay(myLEDTime);
    odomRedLED.set(false);
    odomGreenLED.set(true);
    odomBlueLED.set(false);
    Timer.delay(myLEDTime);
    odomRedLED.set(false);
    odomGreenLED.set(false);
    odomBlueLED.set(true);
    Timer.delay(myLEDTime);
    */

    // RGB LED functions elegant

    /*
    odomRGBBlink(true, false, false, 0.5, 0.25);     // red, green, blue, onTime offTime
    odomRGBBlink(false, true, false, 0.5, 0.25);     // red, green, blue, onTime offTime
    odomRGBBlink(false, false, true, 0.5, 0.25);     // red, green, blue, onTime offTime
    odomRGBBlink(true, true, false, 0.5, 0.25);     // red, green, blue, onTime offTime
    odomRGBBlink(false, true, true, 0.5, 0.25);     // red, green, blue, onTime offTime
    odomRGBBlink(true, false, true, 0.5, 0.25);     // red, green, blue, onTime offTime
    odomRGBBlink(true, true, true, 0.5, 0.25);     // red, green, blue, onTime offTime
    */

    // button stuff:
    // read and print the buttpon state
    boolean buttonValue = odomButton.get();
    System.out.println("buttonValue = " + buttonValue);

    if(buttonValue) {
      // red:
      odomRGB(true, false, false);     // red, green, blue, onTime offTime
    }
    else {
      // blue:
      odomRGB(false, false, true);     // red, green, blue, onTime offTime
    }

    // Sharp IR stuff:
    // see WPILib >> Hardware >> Sensors >> Analog Inputs - Software

    // set the number of averaging bits to 4.  The following command is OPTiONAL!
    odomIRRanger.setAverageBits(4);                     // OPTIONAL 4 bit averaging = 16 samples added together and averaged
    
    // read and print the raw (averaged) IR Value:  
    int rawIRValue = odomIRRanger.getValue();           // 12-bit resolution (0-4096)
    System.out.println("rawIRValue = "+ rawIRValue);    // 

    // beep if object gets too close (2000 = 10cm)!
    if (rawIRValue > 2000) {
      odomBuzzer.set(true);
      // the following is necessary if you have code that will run for a while after this if-statement
      Timer.delay(0.25);
      odomBuzzer.set(false);
    }
    else {
      odomBuzzer.set(false);
    }
/*

    // servo stuff:
    // ----------- SPRINGRC Servo ------------
    // Code for continuous rotation SpringRC servo using the .set() method:
   odomSRCServo.set(0);		// max CW speed
   Timer.delay(1);		// give servo time to move
   odomSRCServo.set(0.5);	// stop
   Timer.delay(0.5); 		// pause time
   odomSRCServo.set(1);		// max CCW speed
   Timer.delay(1); 		// give servo time to move
   odomSRCServo.set(0.5);	// stop
   Timer.delay(0.5); 		// pause time


    // Code for continuous rotation SpringRC servo using the .setAngle() method
    // spoiler alert -- it does the same thing, just with different arguments
   odomSRCServo.setAngle(0);	// max CW speed
   Timer.delay(1);		// give servo time to move
   odomSRCServo.setAngle(90);	// stop
   Timer.delay(0.5); 		// pause time
   odomSRCServo.setAngle(180);	// max CCW speed
   Timer.delay(1); 		// give servo time to move
   odomSRCServo.setAngle(90);	// stop
   Timer.delay(0.5); 		// pause time


    // this scrolls through a wide range ofspeeds for a continuous rotation servo
    for (double i = 55; i <= 125; i++){
      odomSRCServo.setAngle(i);
      Timer.delay(0.2);
    }
    odomSRCServo.setAngle(90);        // stop
    Timer.delay(1);

      // ----------- HITEC Servo ------------
        // code for unmodified 180-degree servos is practically the same as modified 
    // 360-degree servos, but watch the timing:
   odomHitecServo.setAngle(0);	// move to max CW position and stay there
   Timer.delay(2);
   odomHitecServo.setAngle(90);	// move to middle position and stay there
   Timer.delay(2);
   odomHitecServo.setAngle(180);	// move to max CCW position and stay there
   Timer.delay(2);

   // this scrolls slowly through all the positions for an unmodified servo
   odomHitecServo.setAngle(0);	// move to max CW position and stay there
   Timer.delay(1);
   for (double i = 0; i <= 180; i++){
      odomHitecServo.setAngle(i);
      Timer.delay(0.02);
   }
    Timer.delay(1);
*/

// Xbox stuff
// make sure the tiny switch in the back of the Logitec Xbox Controller is set to X, not D
// otherwise the X, Y, A, B buttons are wacky (X=A, Y=Y, A=B, B=X)
System.out.println("-----------------------");
System.out.println("");
// brute force methods for the ten Xbox buttons (boolean) 
System.out.println(odomXbox.getAButton());              // (boolean) true when GREEN A button is being depressed
System.out.println(odomXbox.getBButton());              // (boolean) true when RED B button is being depressed
System.out.println(odomXbox.getXButton());              // (boolean) true when BLUE X button is being depressed
System.out.println(odomXbox.getYButton());              // (boolean) true when YELLOW Y button is being depressed
System.out.println(odomXbox.getLeftBumper());           // (boolean) true when left bumper is being depressed
System.out.println(odomXbox.getRightBumper());          // (boolean) true when right bumper is being depressed
System.out.println(odomXbox.getLeftStickButton());      // (boolean) true when left Joystick is being depressed
System.out.println(odomXbox.getRightStickButton());     // (boolean) true when right Joystick is being depressed
System.out.println(odomXbox.getBackButton());            // (boolean) true when back button is being depressed
System.out.println(odomXbox.getStartButton());           // (boolean) true when start button is being depressed

// brute force methods for Xbox the nine POV positions (int as degrees, -1 as default unused) 
System.out.println(odomXbox.getPOV());                  // (int) get POV value (default = -1, then in degrees from 0 to 315 in 45 degree increments running CW from 12 o'clock on the face.)

// brute force methods for four Xbox axis switches (joysticks and triggers) (double) 
System.out.println(odomXbox.getLeftX());                // (double) left joystick, x channel value ([-1 to 1])
System.out.println(odomXbox.getLeftY());                // (double) left joystick, y channel value ([-1 to 1])
System.out.println(odomXbox.getRightX());               // (double) right joystick, x channel value ([-1 to 1])
System.out.println(odomXbox.getRightY());               // (double) right joystick, y channel value ([-1 to 1])
System.out.println(odomXbox.getLeftTriggerAxis());      // (double) left trigger, value ([0 to 1])
System.out.println(odomXbox.getRightTriggerAxis());     // (double) right trigger, value ([0 to 1])

// Held vs Pressed vs Released for buttons:
System.out.println(odomXbox.getBButton());                // (boolean) true when RED B button is being depressed
System.out.println(odomXbox.getBButtonPressed()   );      // (boolean) RED B button - only true when initially pressed
System.out.println(odomXbox.getBButtonReleased()  );      // (boolean) RED B button - only true when released

// two ways to act on a button press
odomBuzzer.set(odomXbox.getBButton());        // beep if the B button (index #2 is) is pressed
odomBuzzer.set(odomXbox.getRawButton(2) );    // beep if the B button (index #2 is) is pressed
  
  
// button index map for easier polling:
// button indexes begin at 1 not 0 in WPILib for Java and C++
String[] buttonName = {"n/a", "A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LJoy", "RJoy"};

for (int i=1; i < 11; i++) {
  System.out.println(i + ". " + buttonName[i]+ " = " + odomXbox.getRawButton(i) );

  if(odomXbox.getRawButton(i) ) System.out.println("The " + buttonName[i] + " has been pressed!");
}

System.out.println("----------------------");

// axis index map for easier polling:
// great diagnostic! NB: at neutral position, joysticks are not always (0,0)!!!
String[] axisName = {"Lx", "Ly", "Ltrig", "Rtrig", "Rx", "Ry"};

for (int i=0; i < 6; i++) {
  System.out.println(i + ". " + axisName[i]+ " = " + odomXbox.getRawAxis(i) );
}


System.out.println("");
System.out.println("----------------------");
// Timer.delay(2);      // use this to delay so you can see the print out

// PWM Spark Max (not CAN) stuff with Catherine
// the left motor (m_leftDrive) is created by default and set to PWM pin #0
/*
// deadreckon code to spin CCW, stop, spin CCW, stop
m_leftDrive.set(0.25);      // argument = speed as % output of the motor, between -1 and 1.  + is CCW, - is CW, 0 = stop
Timer.delay(1);
m_leftDrive.set(0.0);      // argument = speed as % output of the motor, between -1 and 1.  + is CCW, - is CW, 0 = stop
Timer.delay(0.2);
m_leftDrive.set(-0.25);      // argument = speed as % output of the motor, between -1 and 1.  + is CCW, - is CW, 0 = stop
Timer.delay(1);
m_leftDrive.set(0.0);      // argument = speed as % output of the motor, between -1 and 1.  + is CCW, - is CW, 0 = stop
Timer.delay(0.2);
*/

// drive the motor with the Xbox controller (left joystick y)
m_leftDrive.set(odomXbox.getLeftY());

}

  public void odomRGB(boolean redState, boolean greenState, boolean blueState) {
    odomRedLED.set(redState);
    odomGreenLED.set(greenState);
    odomBlueLED.set(blueState);
  }

  public void odomRGBBlink(boolean redState, boolean greenState, boolean blueState, double onTime, double offTime) {
    odomRedLED.set(redState);
    odomGreenLED.set(greenState);
    odomBlueLED.set(blueState);
    Timer.delay(onTime);
    odomRedLED.set(false);
    odomGreenLED.set(false);
    odomBlueLED.set(false);
    Timer.delay(offTime);
  }
}

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
// import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
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
  // XboxController odomXbox = new XboxController (0);

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
// System.out.println(XboxController.Button.kX.value);

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

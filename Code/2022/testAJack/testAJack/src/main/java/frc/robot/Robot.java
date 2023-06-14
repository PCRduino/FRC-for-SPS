// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// odom imports:
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

// Jack imports
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();

  // odom constructors:
  DigitalInput odomButton = new DigitalInput(0); // DIO pin 0
  DigitalOutput odomLED = new DigitalOutput(1); // DIO pin 1
  AnalogInput odomIRRanger = new AnalogInput(0); // analog pin 0
  DigitalOutput odomServo = new DigitalOutput(9); // servo on DIO 9 (not PWM!!!)
  Servo odomSpringRCServo = new Servo(9); // SpringRC continuous on PWM pin 9
  Servo odomHitecServo = new Servo(7);
  CANVenom canMotor1 = new CANVenom(1); // argument is motor ID number. get this by web browser: 10.15.12.2:5812

  int myServoPosition = 0; // fully CCW position on Hitec servo

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("robotInit called for odom!");

    // odom
    // super(0.03); // change the time of update from 20ms (0.02) to xxx

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
    System.out.println("autoInit called for odom!");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // System.out.println("autoPeriodic called for odom!");
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    System.out.println("teleopInit called for odom!");
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // System.out.println("teleopPeriodic called for odom!");
    // m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    System.out.println("testInit called for odom!");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    System.out.println("testPeriodic called for odom!" + Timer.getMatchTime());

    // read an exrternal button board
    boolean buttonValue = odomButton.get();

    System.out.println("buttonValue = " + buttonValue);

    // // turn on an LED based on the button state
    // if (buttonValue) {
    //   odomLED.set(false);
    // } else {
    //   odomLED.set(true);
    // }

    // a much easier way, suggested by xxx:
    odomLED.set(!buttonValue);

    // controlling the Venom motor with CAN
    canMotor1.enable(); // not sure if necessary???
    double camM1Temp = canMotor1.getTemperature();
    double camM1Pos = canMotor1.getPosition();
    System.out.println("camM1Temp = " + camM1Temp);
    System.out.println("camM1Pos = " + camM1Pos);

    // print all the control modes:
    for (CANVenom.ControlMode c : CANVenom.ControlMode.values())
      System.out.println(c);

    // canMotor1.SetCommand( ControlMode::kSpeedControl, 1000);

    Timer.delay(3);

    odomSpringRCServo.set(0); // move SpringRC servo to max CW
    Timer.delay(1);
    odomSpringRCServo.set(0.5); // stop SpringRC servo:
    Timer.delay(1);
    odomSpringRCServo.set(1); // move SpringRC servo to max CCW
    Timer.delay(1);
    odomSpringRCServo.set(0.5); // stop SpringRC servo:
    Timer.delay(1);

    // contro the HiTec servo:
    if (myServoPosition == 0) {
      myServoPosition = 180;
    } else {
      myServoPosition = 0;
    }

    odomHitecServo.setAngle(myServoPosition); // move Hitec servo to fully CCW (0), [180 = fully CW] -- rudy

    // sweep through all servo positions fgor SpringRC:
    for (double i = 0; i <= 1.0; i += 0.001) {
      odomSpringRCServo.set(i);
      Timer.delay(0.005);
    }

    /*
     * this odom homemade servo library didn't work well. i think the RoboRio is
     * just too
     * slow to control PWM using code. i imagine interrupts are required in some
     * library. see ABOVE for more info...
     */

    // PW constants:
    double fullForward = 0.002003;
    double servoStop = 0.001500;
    double fullReverse = 0.000999;

    // test servo using homemade PWM code:
    System.out.println("forward 1    " + Timer.getFPGATimestamp());
    for (int i = 0; i < 30; i++) {
      // create a 2000us pulse:
      odomServo.set(false); // low
      Timer.delay(0.020); // 20ms delay
      odomServo.set(true); // high
      Timer.delay(fullForward); // full forward
    }

    // another homemade servo test:
    System.out.println("stop 1    " + Timer.getFPGATimestamp());
    odomSpinServo(servoStop, 90); // full reverse
    System.out.println("reverse 1    " + Timer.getFPGATimestamp());
    odomSpinServo(fullReverse, 90); // full reverse
    System.out.println("stop 2    " + Timer.getFPGATimestamp());
    odomSpinServo(servoStop, 90); // full reverse
    System.out.println("forward 2    " + Timer.getFPGATimestamp());
    odomSpinServo(fullForward, 30); // full reverse
    System.out.println("stop 3    " + Timer.getFPGATimestamp());
    odomSpinServo(servoStop, 90); // full reverse
    System.out.println("delay 3.000    " + Timer.getFPGATimestamp());
    Timer.delay(3.0);

    // servo sweep:
    for (double i = fullReverse; i < fullForward; i += 0.000001) {
      System.out.println(i + ", " + Timer.getFPGATimestamp());
      odomCreatePulse(i);
    }
    for (double i = fullForward; i < fullReverse; i -= 0.000001) {
      System.out.println(i + ", " + Timer.getFPGATimestamp());
      odomCreatePulse(i);
    }

    // read a Sharp IR sensor (see WPLib "Analog Inputs - Software"):
    int irValue = odomIRRanger.getValue(); // gets the unaveraged value from the sensor
    System.out.println("irValue = " + irValue);

    // set the averaging "rate". For example, this is 4-bit averaging
    // so 16 samples will be added together and averaged:
    odomIRRanger.setAverageBits(4);

    int irValueAvg = odomIRRanger.getValue(); // gets the unaveraged value from the sensor
    System.out.println("irValueAvg = " + irValueAvg);

    // the robot running time:
    double roborRunTime = Timer.getFPGATimestamp();
    System.out.println("roborRunTime = " + roborRunTime);

    System.out.println("");

    // pause so we can read the output better:
    // Timer.delay(2.5); // delay for this many SECONDS -- you'll get warnings
    // because the code can't update as expected!
  }

  private void odomSpinServo(double PW, int numSteps) {
    for (int i = 0; i < numSteps; i++) {
      odomCreatePulse(PW);
    }
  }

  private void odomCreatePulse(double PW) {
    // create a user-defined pulse:
    // odomServo.set(false);
    // Timer.delay(0.01); // make sure the servo is set low
    odomServo.set(false); // low
    Timer.delay(0.020); // 20ms delay
    odomServo.set(true); // high
    Timer.delay(PW); // full forward

  }

}

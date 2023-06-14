// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.Solenoid.Value
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogGyro;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.ControlMode;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Robot extends TimedRobot {
  // default constructors

  private final Timer m_timer = new Timer();

  XboxController driveController = new XboxController(1); // plug in Xbox controller to USB port on laptop. See Driver
                                                          // Station
  // >> USB tab for the port number (probably 0)
  XboxController armController = new XboxController(0); // plug in Xbox controller to USB port on laptop. See Driver
                                                        // Station
  // >> USB tab for the port number (probably 1)
  // odom commented out:
 //DoubleSolenoid conePiston = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 0); // left close
 //DoubleSolenoid cubePiston = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 3, 2); // 7,6 are pins for open and close

DoubleSolenoid armPiston = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 6); // 7,6 are pins for open and FIX close

 // ODOM code:                    
DoubleSolenoid leftGripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1); // PCM pins 0 and 1
DoubleSolenoid rightGripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);   // PCM pins 2 and 3






  Solenoid stopingPiston = new Solenoid(PneumaticsModuleType.CTREPCM, 4);    // Yellow Y-Button --- Hey HP+ students, please comment your code!

  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  CANVenom mecanumFL = new CANVenom(2); // Motors for mecanum drive
  CANVenom mecanumFR = new CANVenom(3); // Motors for mecanum drive
  CANVenom mecanumBL = new CANVenom(1); // Motors for mecanum drive
  CANVenom mecanumBR = new CANVenom(4); // Motors for mecanum drive

  CANVenom tankL = new CANVenom(2); // Motors for tank drive
  CANVenom tankR = new CANVenom(3); // Motors for tank drive

  PWMSparkMax leadScrewMotor = new PWMSparkMax(0); // Motor that move the lead screw
  DigitalInput leadScrewLimitSwitchMax = new DigitalInput(8); // Stop the motor from spinning outward
  DigitalInput leadScrewLimitSwitchMin = new DigitalInput(9); // Stop the motor from spinning inward

  private boolean isTankDrive = true; // Change to switch drive mode
  private int startPosition = 2; // 1, 2 or 3

  @Override
  public void endCompetition() {

  }

  private ADXRS450_Gyro spiGyro = new ADXRS450_Gyro();

  @Override
  public void robotInit() {

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    System.out.println("Starting Auton");

    // closeCone();        // odom commented out

    stopingPiston.set(false);

    autonSection = 1;
  }

  /** This function is called periodically during autonomous. */

  private int autonSection = 1;

  @Override
  public void autonomousPeriodic() {

    // System.out.println("LeadScrewMAx: " + leadScrewLimitSwitchMax.get());
    System.out.println("Auton Section: " + autonSection);

    if (autonSection == 1) {
      if (leadScrewLimitSwitchMax.get()) {
        armPiston.set(Value.kReverse);
        leadScrewMotor.set(1);
      } else {
        autonSection = 2;
      }
    }
    if (autonSection == 2) {
      if (leadScrewLimitSwitchMin.get()) {
        // openClaw();         //  odom commented out
        tankL.setCommand(ControlMode.Proportional, -0.3);
        tankR.setCommand(ControlMode.Proportional, -0.3);
        leadScrewMotor.set(-1);
      } else {
        autonSection = 4;
        leadScrewMotor.set(0);
        armPiston.set(Value.kForward);
      }
    }
    if (autonSection == 4) {
      if (startPosition == 1) {
        if (m_timer.get() < 7) {
          tankL.setCommand(ControlMode.Proportional, -0.5);
          tankR.setCommand(ControlMode.Proportional, -0.5);
        }
      } else if (startPosition == 2) {
        if (m_timer.get() < 12) {
          tankL.setCommand(ControlMode.Proportional, -0.5);
          tankR.setCommand(ControlMode.Proportional, -0.5);
        } else {
          tankL.setCommand(ControlMode.Proportional, 0.5);
          tankR.setCommand(ControlMode.Proportional, 0.5);
        }
      } else if (startPosition == 3) {
        if (m_timer.get() < 12) {
          tankL.setCommand(ControlMode.Proportional, -0.5);
          tankR.setCommand(ControlMode.Proportional, -0.5);
        } else {
          tankL.setCommand(ControlMode.Proportional, 0.1);
          tankR.setCommand(ControlMode.Proportional, 0.1);
        }
      }
    }

    if (m_timer.get() > 14.5)

    { // REMOVE BEFORE MATCH
      System.out.println("Auton Ended");
      while (true)
        ;
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    compressor.enableDigital();
    CameraServer.startAutomaticCapture();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    updateGameControl();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    compressor.enableDigital();
    CameraServer.startAutomaticCapture();
    // System.out.println("Gyro Connected: " + spiGyro.isConnected());

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    System.out.println("HI!");
    System.out.println("connected: "+spiGyro.isConnected());

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    System.out.println(x+"f "+y+" t "+area);

    // updateGameControl();
  }

  private void updateGameControl() {
    System.out.println("Pressure: " + compressor.getPressure() + "PSI");

    double testingSpeed = 0.4;

    if (isTankDrive) {
      tankDrive(testingSpeed);
    } else {
      mecanumDrive(testingSpeed);
    }

    if (armController.getPOV() == 0) {
      raiseArm();
    } else if (armController.getPOV() == 180) {
      lowerArm();
    }
    System.out.println("Pressure: " + compressor.getPressure() + "PSI");
    stopingPiston.set(false);
    if (armController.getYButton()) {
      stopingPiston.set(true);
    }
    System.out.println(armController.getAButton());
    // odom commented out
    /*
    if (armController.getAButton()) { //good
      closeCube();
    }
    if (armController.getBButton()) { //bad
      closeCone();
    }
    if (armController.getXButton()) { //bad
      openClaw();
    }
    */

    // start odom code:
    System.out.println(armController.getLeftX());
    System.out.println(armController.getRightX());
    if(armController.getLeftX() < -0.8) {
      leftGripper.set(Value.kForward);
    }
    else if(armController.getLeftX() > 0.8) {
      leftGripper.set(Value.kReverse);
    }
    
    if(armController.getRightX() > 0.8) {
      rightGripper.set(Value.kForward);
    }
    else if(armController.getRightX() < -0.8) {
      rightGripper.set(Value.kReverse);
    }

    updateLeadScrewMotor();
  }

  private void raiseArm() {
    armPiston.set(Value.kForward);
    System.out.println("Raise Arm");
  }

  private void lowerArm() {
    armPiston.set(Value.kReverse);
    System.out.println("Lower Arm");
  }

  private void updateLeadScrewMotor() {

    double leadScrewSpeed = 0.5;

    double forwardMotorSpinValue = leadScrewSpeed
        * (leadScrewLimitSwitchMax.get() ? armController.getRightTriggerAxis() : 0);
    double backwardMotorSpinValue = leadScrewSpeed
        * (leadScrewLimitSwitchMin.get() ? armController.getLeftTriggerAxis() * -1 : 0);

    leadScrewMotor.set(forwardMotorSpinValue + backwardMotorSpinValue); // Spins based off of the Left and Right
                                                                        // triggers on the arm controller
  }

  // start odom code:
 private void odomLeftOpen() {

 }



  // end odom code
  

  // odom commented out:
  /*
  private void openClaw() {
    conePiston.set(Value.kForward);       // odom: what is kForward?
    cubePiston.set(Value.kForward);
    System.out.println("Open Claw");

  }

  private void closeCube() {
    cubePiston.set(Value.kReverse);

    System.out.println("Close Cube");
  }

  private void closeCone() {
    conePiston.set(Value.kReverse);

    System.out.println("Close Cone");
  }

  */


  private void tankDrive(double speed) {
    double sL = Math.abs(driveController.getLeftY()) < 0.1 ? 0 : driveController.getLeftY() * speed * -1;
    double sR = Math.abs(driveController.getRightY()) < 0.1 ? 0 : driveController.getRightY() * speed;

    tankL.setCommand(ControlMode.Proportional, sL);
    tankR.setCommand(ControlMode.Proportional, sR);
  }

  private void mecanumDrive(double speed) {
    double turnDirection = driveController.getLeftX();
    double turningSpeed = speed * 0.3;
    double turnAffect = turnDirection * turningSpeed;

    double drivingSpeed = speed * 0.5;

    double x = getXComponentJoyStick();
    double y = getYComponentJoyStick();

    double mFL = (y + x) * drivingSpeed + turnAffect;
    double mBL = (y - x) * drivingSpeed + turnAffect;
    double mFR = -(y - x) * drivingSpeed + turnAffect;
    double mBR = -(y + x) * drivingSpeed + turnAffect;
    // System.out.println(mFL + " " + mBL + " " + mFR + " " + mBR + " ");

    mecanumFL.setCommand(ControlMode.Proportional, mFL);
    mecanumBL.setCommand(ControlMode.Proportional, mBL);
    mecanumFR.setCommand(ControlMode.Proportional, mFR);
    mecanumBR.setCommand(ControlMode.Proportional, mBR);
  }

  private double getXComponentJoyStick() {
    return Math.abs(driveController.getRightX()) > 0.1 ? driveController.getRightX() : 0;
  }

  private double getYComponentJoyStick() {
    return Math.abs(driveController.getRightY()) > 0.1 ? driveController.getRightY() * -1 : 0;
  }
}

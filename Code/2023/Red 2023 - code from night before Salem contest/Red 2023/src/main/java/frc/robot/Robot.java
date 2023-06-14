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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
// import edu.wpi.first.wpilibj.Solenoid.Value
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;

import java.util.ArrayList;
import java.util.DoubleSummaryStatistics;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class Robot extends TimedRobot {
  // default constructors

  private final Timer m_timer = new Timer();

  XboxController driveController = new XboxController(1); // plug in Xbox controller to USB port on laptop. See Driver
                                                          // Station
                                                          // >> USB tab for the port number (probably 0)
  XboxController armController = new XboxController(0); // plug in Xbox controller to USB port on laptop. See Driver
                                                        // Station
  Accelerometer accel = new BuiltInAccelerometer();                                                      // >> USB tab for the port number (probably 1)
  // odom commented out:
  // DoubleSolenoid conePiston = new DoubleSolenoid(0,
  // PneumaticsModuleType.CTREPCM, 1, 0); // left close
  // DoubleSolenoid cubePiston = new DoubleSolenoid(0,
  // PneumaticsModuleType.CTREPCM, 3, 2); // 7,6 are pins for open and close

  DoubleSolenoid armPiston = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 5, 6); // 5,6 are pins for open and FIX
                                                                                        // close

  DoubleSolenoid leftGripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1); // PCM pins 0 and 1
  DoubleSolenoid rightGripper = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3); // PCM pins 2 and 3

  Solenoid armSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4); // Yellow Y-Button --- Hey HP+ students,
                                                                        // please comment your code!

  Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  CANVenom mecanumFL = new CANVenom(2); // Motors for mecanum drive
  CANVenom mecanumFR = new CANVenom(3); // Motors for mecanum drive
  CANVenom mecanumBL = new CANVenom(1); // Motors for mecanum drive
  CANVenom mecanumBR = new CANVenom(4); // Motors for mecanum drive

  CANVenom tankL = new CANVenom(2); // Motors for tank drive
  CANVenom tankR = new CANVenom(3); // Motors for tank drive

  // private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  PWMSparkMax leadScrewMotor = new PWMSparkMax(0); // Motor that move the lead screw
  DigitalInput leadScrewLimitSwitchMax = new DigitalInput(8); // Stop the motor from spinning outward
  DigitalInput leadScrewLimitSwitchMin = new DigitalInput(9); // Stop the motor from spinning inward

  private boolean isTankDrive = true; // Change to switch drive mode

  @Override
  public void endCompetition() {

  }

  private void extend(double v) {

    if (v > 0 && !leadScrewLimitSwitchMax.get()) {
      leadScrewMotor.set(0);
      System.out.println("MAX");
    } else if (v < 0 && !leadScrewLimitSwitchMin.get()) {
      leadScrewMotor.set(0);
      System.out.println("MIN");
    } else {
      leadScrewMotor.set(v);
    }

  }

  

  @Override
  public void robotInit() {

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();

    System.out.println("Starting Auton");

    closeCone();

    armSolenoid.set(true);
  }

  /** This function is called periodically during autonomous. */

  private String phase = "raise";

  @Override
  public void autonomousPeriodic() {

    armSolenoid.set(true);
    // System.out.println("LeadScrewMAx: " + leadScrewLimitSwitchMax.get());
    if (phase == "raise") {
      armPiston.set(Value.kForward);

    }
    if (phase == "raise" && m_timer.get() > 4) {
      m_timer.reset();
      m_timer.stop();
      phase = "extend";
    }
    if (phase == "extend") {
      extend(.75);
    }
    if (!leadScrewLimitSwitchMax.get() && phase == "extend") {
      phase = "drop";
      openClaw();

      m_timer.reset();
      m_timer.start();

    }
    if (phase == "drop" && m_timer.get() > 0.5) {
      extend(-0.5);
      m_timer.reset();
      m_timer.start();
      phase = "backup";
    }

    if (m_timer.get() > 1 && m_timer.get() < 2 && phase == "backup") {
      tankL.setCommand(ControlMode.Proportional, -.4);
      tankR.setCommand(ControlMode.Proportional, .4); // move back
      armPiston.set(Value.kForward);
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    lowerArm();    // DO NOT DELETE THIS!!! (odom 3.1.23)  Needed to ensure the arm does not lift up when it pressurizes on startup.  it will not actually LIFt the arm, but it will set the direction to downward.  needed to pass inspection.
    compressor.enableDigital();
    CameraServer.startAutomaticCapture();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    updateGameControl();
    // System.out.println(leadScrewLimitSwitchMax.get()+"  "+leadScrewLimitSwitchMin.get());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

    // String 
    HttpCamera lmFeed = new HttpCamera("limelight", "http://10.15.12.53:5800/stream.mjpg",HttpCameraKind.kMJPGStreamer );
  compressor.enableDigital();
  // Shuffleboard.getTab("Example").add(gyro);
  // Shuffleboard.getTab("Example").add("Lime Light", lmFeed.)

  
    // CameraServer.startAutomaticCapture();

    initStabilization();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
    // SmartDashboard.putNumberArray("Acceletion", new Double[] {accel.getX(), accel.getY(), accel.getZ()});
        
    stablize();
    // System.out.println(gyro.getRotation2d());
    // System.out.println(accel.getX() + ", " + accel.getY() + ", " + accel.getZ());

    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");

    // System.out.println(x + "f " + y + " t " + area);

    // updateGameControl();
  }

  private void updateGameControl() {
    // System.out.println("Pressure: " + compressor.getPressure() + "PSI");

    double testingSpeed = 0.7;

    if(driveController.getAButtonPressed()){
      initStabilization();
    }else
    if(driveController.getAButton()){
      SmartDashboard.putBoolean("Is Stablizing", true);

      stablize();
    }else{
      SmartDashboard.putBoolean("Is Stablizing", false);

      if (isTankDrive) {
        tankDrive(testingSpeed);
      } else {
        mecanumDrive(testingSpeed);
      }
    }

    if (armController.getPOV() == 0) {
      raiseArm();
    } else if (armController.getPOV() == 180) {
      lowerArm();
    }

    armSolenoid.set(false);
    if (armController.getYButton()) {
      armSolenoid.set(true);
    }
    // System.out.println(armController.getAButton());

    // odom commented out

    // if (armCon troller.getXButton()) { //bad
    // openClaw();
    // }else
    // if (armController.getAButton()) { //good
    // closeCube();
    // }else
    // if (armController.getBButton()) { //bad
    // closeCone();
    // }

    // start odom code:
    // System.out.println(armController.getLeftX());
    // System.out.println(armController.getRightX());

    // if (armController.getLeftX() < -0.8) {
    // leftGripper.set(Value.kForward);
    // } else if (armController.getLeftX() > 0.8) {
    // leftGripper.set(Value.kReverse);
    // }

    // if (armController.getRightX() > 0.8) {
    // rightGripper.set(Value.kForward);
    // } else if (armController.getRightX() < -0.8) {
    // rightGripper.set(Value.kReverse);
    // }
    // end of odom code

    // Either moving the left joystick outward or pressing the left button (X) will
    // cause the arm to open
    // Moving it inward or pressing (A) will open

    if (armController.getLeftX() < -0.8 || armController.getAButton()) {
      leftGripper.set(Value.kForward);
    } else if (armController.getLeftX() > 0.8 || armController.getXButton()) {
      leftGripper.set(Value.kReverse);
    }

    // Either moving the right joystick outward or pressing the right button (B)
    // will cause the arm to open
    // Moving it inward or pressing (A) will open

    if (armController.getRightX() > 0.8 || armController.getAButton()) {
      rightGripper.set(Value.kForward);
    } else if (armController.getRightX() < -0.8 || armController.getBButton()) {
      rightGripper.set(Value.kReverse);
    }

    updateLeadScrewMotor();
  }


  double angle = 0.0;
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
  double Vp = 0;
  double Vi = 0;
  double Vd = 0;

  double lastError = 0;

  private void initStabilization(){
    pastAngles = new ArrayList<Double>();
    lastError = 0;

    angle = 0.0;
    //Edit these to change  functionality
    Kp = 0.3; //<--
    Ki = 0.1; //<-- Not used currently
    Kd = 0.1; //<--


    Vp = 0;
    Vi = 0;
    Vd = 0;
  }
  
  ArrayList<Double> pastAngles;

  private double averageFromList(ArrayList<Double> list){
    double avg = 0;
    double len = (double) list.size();
    for(double value: list){
      avg += value/len;
    }
    return avg;
  }

  private void stablize(){

    // System.out.println("Stabilize");
    if(pastAngles.size() >= 10){
      pastAngles.remove(0);
    }

    pastAngles.add(accel.getY());

        // System.out.println("Accel X: " + accel.getX());

        // System.out.println("Accel Y: " + accel.getY());

        // System.out.println("Accel Z: " + accel.getZ());


    double error = 0  - averageFromList(pastAngles); 
    System.out.println("PD Error: " + error);
    // track error over time, scaled to the timer interval
    // Vi = Vi + (error);// * Dt);
    // determine the amount of change from the last time checked
    Vd = (error - lastError) ;/// Dt; 
    // calculate how much to drive the output in order to get to the 
    // desired setpoint. 
   double speed = (Kp * error) + /*(Ki * Vi) +*/ + (Kd * Vd);
    // remember the error for the next time around.
   lastError = error;

    if(isTankDrive){
      if(Math.abs(speed) > 0.01){
        System.out.println("Speed: " + speed);
        tankL.set(speed);
        tankR.set(-speed);
      }else{
        System.out.println("Speed: " + 0);
      }
    }
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
    double leadScrewSpeed = 0.65;

    // SmartDashboard.putBooleanArray("Arm Limits", new boolean[] {leadScrewLimitSwitchMax.get(), leadScrewLimitSwitchMin.get()}]);

    double forwardMotorSpinValue = leadScrewSpeed
        * (leadScrewLimitSwitchMax.get() ? armController.getRightTriggerAxis() : 0);
    double backwardMotorSpinValue = leadScrewSpeed
        * (leadScrewLimitSwitchMin.get() ? armController.getLeftTriggerAxis() * -1 : 0);

    extend(forwardMotorSpinValue + backwardMotorSpinValue); // Spins based off of the Left and Right
                                                            // triggers on the arm controller
  }

  private void openClaw() {
    leftGripper.set(Value.kForward); // odom: what is kForward?
    rightGripper.set(Value.kForward);// henry: built-in varible that does exactly what it sounds like

    System.out.println("Open Claw");
  }

  private void closeCube() {
    rightGripper.set(Value.kReverse);

    System.out.println("Close Cube");
  }

  private void closeCone() {
    leftGripper.set(Value.kReverse);
    rightGripper.set(Value.kReverse);

    System.out.println("Close Cone");
  }

  private void tankDrive(double speed) {

    // System.out.println(tankL.getPosition());
    if(driveController.getLeftBumperPressed()){
      tankL.resetPosition();
      tankR.resetPosition();
      
      // tankL.clearMotionProfilePoints();
      // tankR.clearMotionProfilePoints();
    }else
    if(driveController.getRightBumper() || driveController.getLeftBumper()){
      // tankL.setCommand(ControlMode.Proportional, 0);
      // tankR.setCommand(ControlMode.Proportional, 0);


      // double sL = tankL.getSpeed();
      // tankL.setCommand(ControlMode.SpeedControl, -0.1*sL);
      // double sR = tankR.getSpeed();
      // tankR.setCommand(ControlMode.SpeedControl, -0.1*sR);


      // tankL.setPID(0.1, 0.4, 0.1, 1, 0);
      // tankR.setPID(0.1, 0.4, 0.1, 1, 0);

      // if(tankL.getPosition() < 1){
        tankL.setCommand(ControlMode.PositionControl, 0.5);
      // }
      // if(tankR.getPosition() < 1){

      tankR.setCommand(ControlMode.PositionControl, 0.5
      
      
      );
      // }

      // tankL.stopMotor();
      // tankR.stopMotor();
    }else{
      // tankL.enable();
      // tankR.enable();

      // tankL.setBrakeCoastMode(BrakeCoastMode.Coast);
      // tankR.setBrakeCoastMode(BrakeCoastMode.Coast);

      double sL = Math.abs(driveController.getLeftY()) < 0.1 ? 0 : driveController.getLeftY() * speed * -1;
      double sR = Math.abs(driveController.getRightY()) < 0.1 ? 0 : driveController.getRightY() * speed;

      tankL.setCommand(ControlMode.Proportional, sL);
      tankR.setCommand(ControlMode.Proportional, sR);
    }
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

/*
FRC 2021
Infant Recharge
4734 Monterey Robotics
Cameron Davis and Braden Chilcutt

Controller 1 Driver
  A =
  X =
  Y =
  B = 
  LB = 
  RB =
  LT = 
  RT = 
  LJ = left drive
  RJ = right drive
  START = lock
  BACK = 

Controller 2 Manipulator
  A = intake in
  X = turret left
  Y = intake out
  B = turret right
  LB = low goal
  RB = shoot
  LT = 
  RT = 
  LJ = into shooter
  RJ = into robot
  START = up
  BACK = down
*/

package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import io.github.pseudoresonance.pixy2api.*;
//import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import io.github.pseudoresonance.pixy2api.Pixy2;
//import io.github.pseudoresonance.pixy2api.links.SPILink;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  //private static final String kDefaultAuto = "Default";
  //private static final String kCustomAuto = "My Auto";
  //private String m_autoSelected;
  //private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private DifferentialDrive myRobot;
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private CANSparkMax lowGoal;
  private VictorSPX climb;
  private VictorSPX intake;
  private VictorSPX shooter;
  private VictorSPX intoShooter;
  private VictorSPX turret;
  private TalonSRX preShooter;
  private TalonSRX intoRobot;
  private static final int leftMotorID = 1;
  private static final int rightMotorID = 2;
  private static final int climbID = 3;
  private static final int intakeID = 4;
  private static final int intoRobotID = 5;
  private static final int lowGoalID = 6;
  private static final int shooterID = 7;
  private static final int turretID = 8;
  private static final int intoShooterID = 10;
  private static final int preShooterID = 11;
  private XboxController driverOne;
  private XboxController driverTwo;
  boolean toggleOn = false;
  boolean togglePressed = false;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  private Pixy2 pixycam;
  boolean isCamera = false;
  int state = -1;
  private double xcoord;
  private ArrayList<Block> blocks;
  private double limeLightX;

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    lowGoal = new CANSparkMax(lowGoalID, MotorType.kBrushless);
    intoShooter = new VictorSPX(intoShooterID);
    climb = new VictorSPX(climbID);
    intake = new VictorSPX(intakeID);
    shooter = new VictorSPX(shooterID);
    turret = new VictorSPX(turretID);
    preShooter = new TalonSRX(preShooterID);
    intoRobot = new TalonSRX(intoRobotID);
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    myRobot = new DifferentialDrive(leftMotor, rightMotor);
    driverOne = new XboxController(0);
    driverTwo = new XboxController(1);
    intoShooter.set(ControlMode.PercentOutput, 0);
    climb.set(ControlMode.PercentOutput, 0);
    intake.set(ControlMode.PercentOutput, 0);
    shooter.set(ControlMode.PercentOutput, 0);
    turret.set(ControlMode.PercentOutput, 0);
    preShooter.set(ControlMode.PercentOutput, 0);
    intoRobot.set(ControlMode.PercentOutput, 0);
    pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
    pixycam.init();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    // pixycam.getCCC().getBlocks(false, 255, 255);
    // blocks = pixycam.getCCC().getBlockCache();
    // if (blocks.size() > 0 )
    // {
    //   xcoord = blocks.get(0).getX();
    //   SmartDashboard.putBoolean( "present" , true );
    //   SmartDashboard.putNumber( "Xccord" ,xcoord);
    // }
    // else
    // {
    //   SmartDashboard.putBoolean( "present" , false );
    // }
    // Timer.delay(0.1);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    myRobot.setSafetyEnabled(false);
    myRobot.tankDrive(0.0, 0.0);
    lowGoal.set(0.0);
    intoShooter.set(ControlMode.PercentOutput, 0.0);
    climb.set(ControlMode.PercentOutput, 0.0);
    intake.set(ControlMode.PercentOutput, 0.0);
    shooter.set(ControlMode.PercentOutput, 0.0);
    turret.set(ControlMode.PercentOutput, 0.0);
    preShooter.set(ControlMode.PercentOutput, 0.0);
    intoRobot.set(ControlMode.PercentOutput, 0.0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    Timer.delay(.3);
    limeLightX = tx.getDouble(0.0);
    while(limeLightX > 12.5 || limeLightX < 2.5)
    {
      limeLightX = tx.getDouble(0.0);
      turret.set(ControlMode.PercentOutput, (limeLightX - 7.5)/30);
    }
    turret.set(ControlMode.PercentOutput, 0);

    shooter.set(ControlMode.PercentOutput, 1.0);
    Timer.delay(1.3);
    preShooter.set(ControlMode.PercentOutput, -0.5);
    intoShooter.set(ControlMode.PercentOutput, 0.5);
    intoRobot.set(ControlMode.PercentOutput, -0.5);
    Timer.delay(1.5);
    shooter.set(ControlMode.PercentOutput, 0.0);
    preShooter.set(ControlMode.PercentOutput, 0.0);
    intoShooter.set(ControlMode.PercentOutput, 0.0);
    intoRobot.set(ControlMode.PercentOutput, 0.0);

    myRobot.tankDrive(0.6, 0.6);
    intake.set(ControlMode.PercentOutput, 0.75);
    intoRobot.set(ControlMode.PercentOutput, -0.75);
    intoShooter.set(ControlMode.PercentOutput, 0.65);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    //Timer.delay(2.75);
    pixy();
    Timer.delay(1.5);
    pixy();
    Timer.delay(1.25);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    myRobot.tankDrive(0.0, 0.0);
    Timer.delay(0.5);
    intake.set(ControlMode.PercentOutput, 0.0);
    intoRobot.set(ControlMode.PercentOutput, 0.0);
    intoShooter.set(ControlMode.PercentOutput, 0.0);
    myRobot.tankDrive(-0.6, -0.6);
    Timer.delay(2.0);
    myRobot.tankDrive(0.0, 0.0);
    intoShooter.set(ControlMode.PercentOutput, -.5);
    Timer.delay(0.1);
    intoShooter.set(ControlMode.PercentOutput, 0.0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    Timer.delay(.5);
    limeLightX = tx.getDouble(0.0);
    while(limeLightX > 12.5 || limeLightX < 2.5)
    {
      limeLightX = tx.getDouble(0.0);
      turret.set(ControlMode.PercentOutput, (limeLightX - 7.5)/30);
    }
    turret.set(ControlMode.PercentOutput, 0);

    shooter.set(ControlMode.PercentOutput, 1.0);
    Timer.delay(1.3);
    preShooter.set(ControlMode.PercentOutput, -0.5);
    intoShooter.set(ControlMode.PercentOutput, 0.5);
    intoRobot.set(ControlMode.PercentOutput, -0.5);
    Timer.delay(1.5);
    shooter.set(ControlMode.PercentOutput, 0.0);
    preShooter.set(ControlMode.PercentOutput, 0.0);
    intoShooter.set(ControlMode.PercentOutput, 0.0);
    intoRobot.set(ControlMode.PercentOutput, 0.0);
    myRobot.setSafetyEnabled(true);
    //autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //switch (m_autoSelected) {
      //case kCustomAuto:
        // Put custom auto code here
        //break;
      //case kDefaultAuto:
      //default:
        // Put default auto code here
        //break;
    //}
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic()
  {
    limeLightX = tx.getDouble(0.0);
    myRobot.tankDrive(-driverOne.getY(Hand.kLeft), -driverOne.getY(Hand.kRight));
    intoRobot.set(ControlMode.PercentOutput, driverTwo.getY(Hand.kRight) * -.5);
    intoShooter.set(ControlMode.PercentOutput, -driverTwo.getY(Hand.kLeft) * .5);
    victor(driverTwo, 1, 4, intake, .75);
    victor(driverTwo, 2, 3, turret, .2);
    victor(driverTwo, 8, 7, climb, .75);
    if(driverTwo.getRawButton(5))
      lowGoal.set(-.5);
    else if(driverOne.getRawButton(8))
      lowGoal.set(.5);
    else
      lowGoal.set(0.0);
    if(driverTwo.getRawButton(6))
      shoot();
  }

  public void sparkmax(XboxController c, int bf, int bb, CANSparkMax m, double s)
  {
    if(c.getRawButton(bf))
    {
      m.set(s);
    }
    else if(c.getRawButton(bb))
    {
      m.set(-s);
    }
    else
    {
      m.set(0.0);
    }
  }

  public void victor(XboxController c, int bf, int bb, VictorSPX m, double s)
  {
    if(c.getRawButton(bf))
    {
      m.set(ControlMode.PercentOutput, s);
    }
    else if(c.getRawButton(bb))
    {
      m.set(ControlMode.PercentOutput, -s);
    }
    else
    {
      m.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void shoot()
  {
    myRobot.setSafetyEnabled(false);
    myRobot.tankDrive(0.0, 0.0);
    lowGoal.set(0.0);
    intoShooter.set(ControlMode.PercentOutput, 0.0);
    climb.set(ControlMode.PercentOutput, 0.0);
    intake.set(ControlMode.PercentOutput, 0.0);
    shooter.set(ControlMode.PercentOutput, 0.0);
    turret.set(ControlMode.PercentOutput, 0.0);
    preShooter.set(ControlMode.PercentOutput, 0.0);
    intoRobot.set(ControlMode.PercentOutput, 0.0);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    Timer.delay(.3);
    limeLightX = tx.getDouble(0.0);
    while(limeLightX > 12.5 || limeLightX < 2.5)
    {
      limeLightX = tx.getDouble(0.0);
      turret.set(ControlMode.PercentOutput, (limeLightX - 7.5)/35);
    }
    turret.set(ControlMode.PercentOutput, 0);

    shooter.set(ControlMode.PercentOutput, 1.0);
    Timer.delay(1.3);
    preShooter.set(ControlMode.PercentOutput, -0.5);
    intoShooter.set(ControlMode.PercentOutput, 0.5);
    intoRobot.set(ControlMode.PercentOutput, -0.5);
    Timer.delay(1.5);
    shooter.set(ControlMode.PercentOutput, 0.0);
    preShooter.set(ControlMode.PercentOutput, 0.0);
    intoShooter.set(ControlMode.PercentOutput, 0.0);
    intoRobot.set(ControlMode.PercentOutput, 0.0);
    myRobot.setSafetyEnabled(true);
  }
  public void pixy()
  {
    pixycam.getCCC().getBlocks(false, 255, 255);
    blocks = pixycam.getCCC().getBlockCache();
    if (blocks.size() > 0 )
    {
      xcoord = blocks.get(0).getX();
      SmartDashboard.putNumber( "Xccord" ,xcoord);
    }
    else
    {
      SmartDashboard.putNumber( "Xccoord" , -1 );
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() 
  {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic()
  {

  }
}
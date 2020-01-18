/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.pixy.Pixy2CCC.Block;
import frc.robot.pixy.links.SPILink;
import frc.robot.pixy.Pixy2CCC;
import frc.robot.pixy.Pixy2;
import edu.wpi.first.wpilibj.Joystick;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */

  SPILink SP = new SPILink();
  Pixy2 pixy = Pixy2.createInstance(SP);

  Joystick m_stick;
  Boolean ButtonLB; 


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

      pixy.init();
      pixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
      pixy.setLED(200, 30, 255); // Sets the RGB LED to purple
      System.out.println("testing");
    
    

    m_stick = new Joystick(0);
    ButtonLB = m_stick.getRawButton(6);

		RobotMap.rghtFollower.follow(RobotMap.rghtFront);
		RobotMap.leftFollower.follow(RobotMap.leftFront);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		RobotMap.rghtFront.setInverted(false);
		RobotMap.rghtFollower.setInverted(false);
		RobotMap.leftFront.setInverted(false);
		RobotMap.leftFollower.setInverted(false);

  }

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
    boolean targetFound = false; 
    int Xpos = -1; int Ypos = -1; int Height = -1; int Width = -1; Double objDist = -1.0; double []objAng = {-1.0,-1.0};

    //Getting the number of blocks found
    int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
      System.err.println("No blocks found");
    }
    ArrayList <Block> blocks = pixy.getCCC().getBlocks();
    Block largestBlock = null;

    //verifies the largest block and store it in largestBlock
    for (Block block:blocks){
      if (block.getSignature() == Pixy2CCC.CCC_SIG1) {

				if (largestBlock == null) {
					largestBlock = block;
				} else if (block.getWidth() > largestBlock.getWidth()) {
					largestBlock = block;
        }
			}
    }
    //loop
    while (pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25)>=0 && ButtonLB){

      if (largestBlock != null){
        targetFound = true; 
        Xpos = largestBlock.getX();
        Ypos = largestBlock.getY();
        Height = largestBlock.getHeight();
        Width = largestBlock.getWidth();

        objDist = RobotMap.calculateDist((double)Height, (double)Width, (double)Xpos, (double)Ypos);
        objAng = RobotMap.calculateAngle(objDist, Xpos, Ypos);
        
      }
          //print out values to Dashboard
      SmartDashboard.putBoolean("Target found", targetFound);
      SmartDashboard.putNumber ("Object_X", Xpos);
      SmartDashboard.putNumber ("Object_Y", Ypos);
      SmartDashboard.putNumber ("Height", Height);
      SmartDashboard.putNumber("Width", Width);
      SmartDashboard.putNumber ("Distance", objDist);
      SmartDashboard.putNumber("X Angle", objAng[0]);
      SmartDashboard.putNumber("Y Angle", objAng[1]);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

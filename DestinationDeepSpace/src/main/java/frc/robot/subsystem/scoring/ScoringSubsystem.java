/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.scoring;

import frc.robot.MotorId;
import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.subsystem.scoring.ScoringConstants.BeakPosition;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.utils.talonutils.TalonUtils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;






public class ScoringSubsystem extends BitBucketSubsystem {
	/*
	 * Get the Operator Interface (OI). this allows us to easily communicate with
	 * the joystick(s) and get data (such as level to put arm at) from them
	 */
	private final OI oi = OI.instance();

	// Singleton method; use ScoringSubsystem.instance() to get the ScoringSubsystem instance.
	// note: we might not use singletons in 2020
	public static ScoringSubsystem instance() {
		if(inst == null)
			inst = new ScoringSubsystem();
		return inst;
	}
	/*
	 * singletons = you keep one instance of the class and store it (here it is called inst)
	 * when you need to access the instance you call instance() or equivalent method and it will
	 * either (1) return the instance or (2) make the instance if it hasn't already been made
	 * and then return it.
	 * It's just a fancy way of managing communications between different objects (in our case,
	 * specifically, subsystems (usually)). it's not the only way to go
	*/
	private static ScoringSubsystem inst;


	// variable holding the initial position of the beak to grab hatch panels
	// the robot starts auto with a hatch panel so we initialize it to grab
	// the hatch
	private BeakPosition beakPosition = BeakPosition.HATCH_GRAPPLE_BEAK;


	// start the arm in it's Idle state
	private Idle initialCommand;

	// motor for the roller that rolls ball out of arm and into the cargo ship (hopefully)
	private final WPI_TalonSRX rollerMotor;
	// 2 motors to move the arm up/down because it's just that heavy
	private final WPI_TalonSRX armMotor1;
	private final WPI_TalonSRX armMotor2;
	// motor controlling the beak
	private final WPI_TalonSRX beakMotor;
	// current in amps of both arm motors respectively
	private double armMotor1Current_amps = 0;
	private double armMotor2Current_amps = 0;

	// store the last orientation of the robot's arm in the previous iteration of periodic
	// true --> back
	// false --> front
	// battery in the back
	private boolean back = true;
	// last level the arm was at
	private ScoringConstants.ScoringLevel lastLevel = ScoringConstants.ScoringLevel.NONE;
	// level arm is being commanded to
	private ScoringConstants.ScoringLevel commandedLevel = ScoringConstants.ScoringLevel.NONE;

	// we added a manual arm control, where the driver/operator could manually move the arm
	// up or down instead of using one of the hard coded levels
	// this allows greater flexibility in what the driver team can do, especially if
	// a mechanical error prevents the pre specified level angles from allowing
	// the ball to be scored (this happened once!)
	private double lastManualAngle = 0;


	// get the vision subsystem to coordinate camera behavior with arm orientation
	// we had a bidirectional arm so when it was in the back, we wanted the back camera,
	// and when it was in the front we wanted the front camera
	private VisionSubsystem visionSubsystem = VisionSubsystem.instance();


	// constructor for the scoring subsystem
	private ScoringSubsystem() {
		// for SmartDashboard-ing
		setName("ScoringSubsystem");



		// initialize the motors
		// a better practice is to do this in init. I guess we missed that but luckily
		// didn't run into any problems with it
		rollerMotor = new WPI_TalonSRX(MotorId.INTAKE_MOTOR_ID);
		armMotor1   = new WPI_TalonSRX(MotorId.ARM_MOTOR1_ID);
		armMotor2   = new WPI_TalonSRX(MotorId.ARM_MOTOR2_ID);
		beakMotor   = new WPI_TalonSRX(MotorId.BEAK_MOTOR_ID);

		// initialize motors before setting sensor positions and follower modes
		// otherwise, it may clear those settings
		// Talons also store some of their configurations, so if we have to swap out a Talon then
		// we'll have the wrong ones. so it's best practice to reset to factory defaults and then
		// put in all configs we need manually
		TalonUtils.initializeMotorDefaults(rollerMotor);
		TalonUtils.initializeMotorDefaults(armMotor1);
		TalonUtils.initializeMotorDefaults(armMotor2);
		TalonUtils.initializeMotorDefaults(beakMotor);

		// setting inversions (+input = -voltage applied and vice versa)
		// helpful when two motors run in opposite directions by default
		// but you want them to work together (such as drive motors)
		// the parameter in setInverted() determines whether to invert it
		// and is stored in the ScoringConstants. we like to have a Constants
		// class for every subsystem
		rollerMotor.setInverted(ScoringConstants.ROLLER_MOTOR_INVERSION);
		armMotor1.setInverted(ScoringConstants.ARM_MOTOR_INVERSION);
		armMotor2.setInverted(ScoringConstants.ARM_MOTOR_INVERSION);
		beakMotor.setInverted(ScoringConstants.BEAK_MOTOR_INVERSION);

		// sensor phase = whether to invert the sensor. this depends on how the
		// encoder was put in. just another one of those configs you need
		armMotor1.setSensorPhase(ScoringConstants.ARM_MOTOR_SENSOR_PHASE);
		beakMotor.setSensorPhase(ScoringConstants.BEAK_MOTOR_SENSOR_PHASE);

		// dealing with limit switches: these are buttons that will disable the corresponding motor when pressed
		// this way the arm can't keep running if it's reached the bottom because it will touch the limit switch
		// just a safety feature really
		armMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,0);
		armMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		
		armMotor1.overrideLimitSwitchesEnable(true);

		// tells the motor what to do when there's no input
		// brake = generate a back-emf that forces the motor to counteract any disturbance
		// coast = let disturbances move the motor
		// in this case we want it to stay because gravity will always pull it down
		// for example, with coast on the drive motors, by pushing the robot the wheels will move
		// even if you apply no input
		armMotor1.setNeutralMode(NeutralMode.Brake);
		beakMotor.setNeutralMode(NeutralMode.Brake);


		// PIDF constants
		TalonUtils.initializeMotorFPID(armMotor1, 
							ScoringConstants.ARM_MOTION_MAGIC_KF, 
							ScoringConstants.ARM_MOTION_MAGIC_KP, 
							ScoringConstants.ARM_MOTION_MAGIC_KI, 
							ScoringConstants.ARM_MOTION_MAGIC_KD, 
							ScoringConstants.ARM_MOTION_MAGIC_IZONE);
		
		TalonUtils.initializeMotorFPID(beakMotor, 
							ScoringConstants.BEAK_MOTION_MAGIC_KF, 
							ScoringConstants.BEAK_MOTION_MAGIC_KP, 
							ScoringConstants.BEAK_MOTION_MAGIC_KI, 
							ScoringConstants.BEAK_MOTION_MAGIC_KD, 
							ScoringConstants.BEAK_MOTION_MAGIC_IZONE);

		// add a magnetic encoder to the arm so we can tell the absolute position of the encoder
		// with respect to the vertical position
		// a magnetic encoder can act as absolute or relative, meaning we can get changes
		// in angle or just the angle itself. We initialize it as relative because
		// then it gives us both relative and absolute capabilities
		TalonUtils.initializeMagEncoderRelativeMotor(armMotor1, 1);
		// the beak does need absolute position control too, but mechanically it stays
		// in the same initial position before every match, so we can just use a
		// relative quad encoder for the same result (up to a few ticks of error, but
		// not noticeable or important)
		TalonUtils.initializeQuadEncoderMotor(beakMotor, 1);


		// there are two arm motors (arm is very heavy so it needs twice the torque)
		// follow lets the second arm motor copy the movements of the first so they're
		// always working together in sync
		armMotor2.follow(armMotor1);

		
		
		// Acceleration is the slope of the velocity profile used for motion magic
		// Example: 250 tick/100ms/s is 2500 ticks/s/s
		armMotor1.configMotionAcceleration(ScoringConstants.ARM_ACCELERATION_TICKS_PER_100MS_PER_SEC, 20);
		armMotor1.configMotionCruiseVelocity(ScoringConstants.ARM_CRUISE_SPEED_TICKS_PER_100MS, 20);

		beakMotor.configMotionAcceleration(ScoringConstants.BEAK_ACCELERATION_TICKS_PER_100MS_PER_SEC, 20);
		beakMotor.configMotionCruiseVelocity(ScoringConstants.BEAK_CRUISE_SPEED_TICKS_PER_100MS, 20);
		


		// does what it says, disables all motors
		setAllMotorsZero();




		// get the absolute ticks of the arm
		// the ARM encoder has a resolution of 4096 ticks (drive has a different type),
		// so 1 revolution can be between 0ticks to 4095ticks
		// just using .getSelectedSensorPosition() will return RELATIVE (remember,
		// we initialized the encoder as relative, but it also has absolute
		// capabilities), so we need more data from the sensor to get absolute
		// We use the Talon's SensorCollection and get the position from there
		// then we mod (remainder of a division with) 4095 in case we get a higher value than
		// expected
		// for example, 0 degrees represents the same position as 360 degrees or 720 degrees
		// but we want to have it between -180 and 180, so we have to do some division by 360
		// for example, 725 mod 360 = 5 (because 360*2 + 5 = 725) and we get 5 degrees
		// & is a binary operator that does the same as % in this case due to the nature of
		// binary and 4096 being a power of two.
		// Don't worry if you don't understand the binary part of it, we will probably
		// be more clear and use % 4095 if we need to do this next year
		int abs_ticks = armMotor1.getSensorCollection().getPulseWidthPosition() & 0xFFF;
		// set the ticks of relative magnetic encoder
		// effectively telling the encoder where 0 is
		// a bias accounts for a few things:
		// 1. mounting of the encoder: for example, if the encoder is rotated so that what should
		//    be read as 0 deg is read as 10 deg, we subtract the bias of 10 deg to get the
		//    value we want
		// 2. normalization: by this, I mean the interval the encoder reads on. For example,
		//    the encoder can return 0 deg to 360 deg with 180 being vertical
		//    but we want [-180, 180] so we can subtract 180 degrees and get the desired interval
		// we take the absolute position and subtract the bias, then set the RELATIVE position
		// (because the selected sensor was initialized as relative) to this.
		// relative effectively gives a displacement by default, unless you specify an initial
		// position. We want to know the absolute position but want to use the relative encoder
		// to get it (easier code, don't have to renormalize, etc...) so we set the initial position
		// to what it should be so we can use relative
		armMotor1.setSelectedSensorPosition(abs_ticks - ScoringConstants.ARM_BIAS_TICKS);
		// The beak is normally at rest during initialization, this makes sure the motor knows that.
		beakMotor.setSelectedSensorPosition(0);



		// for some mysterious reason that we never figured out, the encoder still wasn't
		// reading angles between -180 and 180
		// we found out that if we have it wait for about half a millisecond, then it will
		// start giving accurate values (maybe we should've done all this in init?)
		// Thread.sleep(500) waits 500ms = 0.5sec before updating the encoder measurement
		try {
// 254
			Thread.sleep(500);
		} catch (InterruptedException e) {}

		// get the angle
		double angle = getAngle_deg();
		// normalize it to be in [-180, 180]
		if (angle <= -180) {
			angle += 360;
		}
		else if (angle >= 180) {
			angle -= 360;
		}

		// set the new angle for good now
		setAngle_deg(angle);
	}





	/*
	 * I drew this for a method I realized we didn't even need but decided to keep it, enjoy!
	 * 
	 * 
	 *        \   /            \   /                               \   /            \   /
	 *         \_/              \_/               G O               \_/              \_/
 	 *          \\               \\                                 //               //
	 *           \\               \\              BIT              //               //
	 *       _____\\_____     _____\\_____                   _____//_____     _____//_____
	 *       |          |     |          |      BUCKETS      |          |     |          |
	 *      ==O========O==   ==O========O==                 ==O========O==   ==O========O==
	 */


	
	// Put methods for controlling this subsystem
	// here. Call these from Commands.


	/**
	 * Get the level that is currently being commanded
	 * 
	 * @return commanded level
	 */
	public ScoringConstants.ScoringLevel getCommandedLevel()
	{
		return commandedLevel;
	}

	/**
	 * Command the arm to a pre-defined level
	 * @param level level to command arm to
	 */
	public void goToLevel(ScoringConstants.ScoringLevel level) {
		// set the new commanded level
		commandedLevel = level;

		// neither level should get to here in the first place
		//     ... but just in case
		// we do checks like these to make our code more ~robust~
		if (
			level == ScoringConstants.ScoringLevel.NONE ||
			level == ScoringConstants.ScoringLevel.INVALID
		) {
			// just quit the method
			return;
		// if using manual control...
		} else if (level == ScoringConstants.ScoringLevel.MANUAL){
			// using the manual control "level"
			// when we first came up with the level system, we didn't
			// think we'd have to add manual control (good thing to
			// consider as we're coding in 2020!)
			// so we had a hard time incorporating it into the code
			// making MANUAL a new level was the best we could
			lastLevel=ScoringConstants.ScoringLevel.MANUAL;
			// angle manual control is currently at
			lastManualAngle = getAngle_deg();
			// don't go to regular level control
			return;
		}


		// get the angle the level corresponds to
		double angle_rad = level.getAngle_rad();

		// .switchOrientation() needs to know the last level the arm was at
		// if not, then whether the arm should be in the front or back is useless
		// because we have no way to recommand it to the right level
		lastLevel = level;

		// command the corresponding angle
		directArmTo(angle_rad);
	}



	/**
	 * Command a percent output the roller motor. It doesn't need to be an exact
	 * velocity or anything, as long as its
	 * (1) enough to get the ball out and score
	 * (2) not too much that it compresses and deforms the ball resulting in a penalty
	 * 
	 * @param percent output to command rollers to
	 * + pow --> spit out
	 * - pow --> intake
	 */
	public void setRollers(double pow) {
		// may be the other way around depending on the placement of the motors and such

		/*
		 *    <---  __
		 *         /  \
		 *         \__/  --->
		 * 
		 *         O (ball) (NOT TO SCALE) -->
		 * 
		 *          __   --->
		 *         /  \
		 *    <--- \__/
		 */
		// just command the power output
		// we could call this line of code whenever we need to run the rollers but
		// making it a method is less verbose and more intuitive
		rollerMotor.set(ControlMode.PercentOutput, pow);
	}



	/* stop all current subsystem functions (used in Idle) */
	public void disable() {
		setAllMotorsZero();
	}



	/* switch the orientation of the arm */
	public void switchOrientation() {
		// can't switch if the beak up is because gravity will actually
		// cause the hatch to fall
		// the arm itself is bidirectional but its hatch functionality
		// isn't - we added the beak during competition season because
		// we found that our original plan of having the rollers spit
		// out the hatch panels wasn't working
		if (beakPosition == BeakPosition.HATCH_GRAPPLE_BEAK) {
			// prevent any switching if its up
			return;
		}

		// reverse the field that indicates the orientation of the arm
		// we use this for determining which camera to enable
		back = !back;

		// enable respective camera from the vision subsystem
		// this allows the drive team to see the relevant
		// side of the robot (the one we're scoring with)
		if (back)
		{
			visionSubsystem.enableBack();
		}
		else
		{
			visionSubsystem.enableFront();
		}

		// go to the last level the arm was at, but this time
		// with the new orientation (handled by the method)
		// There are two positions per level - front and back, so make sure
		// to switch to the right level
		goToLevel(lastLevel);
	}





	/** Get the selected level on the joystick */
	// Used so much in commands that I just put it in the subsystem
	public ScoringConstants.ScoringLevel getSelectedLevel() {
		// get which levels are selected
		boolean hp = oi.hp(); // hatch panel
        boolean ground = oi.ground(); // ground level for ball pickup
        boolean bCargo = oi.bCargo(); // cargo level for ball scoring
        boolean bLoadingStation = oi.bLoadingStation(); // loading station level for ball pickup
		boolean bRocket1 = oi.bRocket1(); // rocket stage 1 level for ball scoring
		boolean topDeadCenter = oi.topDeadCenter(); // vertical position
		double manual = oi.manualArmControl(); // get the manual control input, dictating how much
		                                       // to manually move arm by
		
		// default to no selected level
		ScoringConstants.ScoringLevel level = ScoringConstants.ScoringLevel.NONE;

		// the purpose of this convoluted chain of if statements is to make sure
		// only one level is selected at once
		// if multiple are selected, it will give an invalid level and nothing will happen

		// starts off checking if one level is selected
		// then, checks the others to see which is selected
		// for every selected level, it will make a choice.
		//   if no level has been selected in the code (not joystick) yet, it will select
		//       it
		//   if another level has been selected, then 2+ levels have been selected so invalidate
		//       the level

		// deadband = region in which we ignore inputs
		// sometimes the joystick gives inaccurate readings so we ignore inputs inside of a deadband
		// if the manual control outside of the deadband (aka |input| > deadband), then we're using
		// manual
		if (manual > ScoringConstants.ARM_MANUAL_DEADBAND || manual < -ScoringConstants.ARM_MANUAL_DEADBAND) {
			level = ScoringConstants.ScoringLevel.MANUAL;
		}
		// check other levels using above procedure
		if (hp) {
			if (level == ScoringConstants.ScoringLevel.NONE)
			{
				level = ScoringConstants.ScoringLevel.HP;
			}
			else
			{
				level = ScoringConstants.ScoringLevel.INVALID;
			}
		}
		if (topDeadCenter)
		{
			if (level == ScoringConstants.ScoringLevel.NONE)
			{ 
				level = ScoringConstants.ScoringLevel.TOP_DEAD_CENTER; 
			}
			else 
			{ 
				return ScoringConstants.ScoringLevel.INVALID; 
			}
		}
		if (ground) {
			if (level == ScoringConstants.ScoringLevel.NONE)
			{ 
				level = ScoringConstants.ScoringLevel.GROUND; 
			}
			else 
			{ 
				return ScoringConstants.ScoringLevel.INVALID; 
			}
		}
		if (bCargo) {
			if (level == ScoringConstants.ScoringLevel.NONE) { level = ScoringConstants.ScoringLevel.BALL_CARGO; }
			else { return ScoringConstants.ScoringLevel.INVALID; }
		}
		if (bLoadingStation) {
			if (level == ScoringConstants.ScoringLevel.NONE) { level = ScoringConstants.ScoringLevel.BALL_LOADING_STATION; }
			else { return ScoringConstants.ScoringLevel.INVALID; }
		}
		if (bRocket1) {
			if (level == ScoringConstants.ScoringLevel.NONE) { level = ScoringConstants.ScoringLevel.BALL_ROCKET_1; }
			else { return ScoringConstants.ScoringLevel.INVALID; }
		}

		// return the result
		return level;
	}



	/**
	 * Get the absolute error of the arm's position in ticks
	 * 
	 * @return absolute error in ticks
	 */
	public int getArmLevelTickError() {
		// get the magnitude of the closed loop error = error in the closed loop control
		int err1 = Math.abs(armMotor1.getClosedLoopError());

		// Always output this
		//SmartDashboard.putNumber(getName() + "/ArmLevelError (ticks)", err1);
		// it says "always output this" but then we got warned by a judge that we used too
		// much bandwidth in the NetworkTables so we had to remove it

		// return the error
		return err1;
	}





	// Internal control of subsystem



	/** Trivial, comments are left to the reader as an exercise :)))) */
	private void setAllMotorsZero() {
		rollerMotor.set(ControlMode.PercentOutput, 0);
		armMotor1.set(ControlMode.PercentOutput, 0);
		beakMotor.set(ControlMode.PercentOutput, 0);
	}



	// private because we only want other classes to change the angle via goToLevel()
	// angle we want to move the arm to
	private double targetAngle_rad = 0.0;
	/**
	 * Return the current target angle
	 * 
	 * @return current target angle
	 */
	public double getTargetAngle_rad()
	{
		return targetAngle_rad;
	}

	/**
	 * Direct the robot arm to a certain angle.
	 * 
	 * @param angle_rad angle, in radians, to direct the arm to. This should be positive
	 *                  as it calculates the actual angle based on this and the
	 *                  arm's current orientation
	 */
	public void directArmTo(double angle_rad) {
		// update the target angle
		targetAngle_rad = angle_rad;
		// convert to CTRE units
		double ticks = angle_rad * ScoringConstants.ARM_MOTOR_NATIVE_TICKS_PER_REV / (2 * Math.PI);

		// if the arm is in the back of the robot
		if (back == false) {
			// switch the ticks so that the arm will go to intended position on the back too
			// ticks = 0 means arm is just up
			ticks *= -1;
			targetAngle_rad *= -1;
			// output the commanded angle to the dashboard
			//SmartDashboard.putNumber(getName()+"/Arm Command Angle (deg)", Math.toDegrees(-angle_rad));

		}
		else
		{
			// output the commanded angle to the dashboard
			//SmartDashboard.putNumber(getName()+"/Arm Command Angle (deg)", Math.toDegrees(angle_rad));
		}

		// command it using MotionMagic
		armMotor1.set(ControlMode.MotionMagic, ticks);
	}



	/** Get angle from normal (vertical) of scoring arm (-90 deg = exactly forward, +90 is backward) */
	public double getAngle_deg() {
		// get angle in ticks
		int ticks = armMotor1.getSelectedSensorPosition();

		// then convert to useful units (degrees)
		return 360.0 * (double)ticks / (double)ScoringConstants.ARM_MOTOR_NATIVE_TICKS_PER_REV;
	}



	/**
	 * Set the angle of the arm in degrees
	 * @param deg angle to set in degrees
	 */
	public void setAngle_deg(double deg) {
		// convert to ticks that encoder uses
		int ticks = (int) (ScoringConstants.ARM_MOTOR_NATIVE_TICKS_PER_REV * deg / 360);

		// then set the position
		armMotor1.setSelectedSensorPosition(ticks);
	}
	



	
	/**
	 * Start the subsystem's Idle command - the initial command in the state machine
	 */
	public void startIdle() {
		// Don't use default commands as they can catch you by surprise
		System.out.println("Starting " + getName() + " Idle...");
		// only start an initial command if one hasn't already been made
		if (initialCommand == null) {
			initialCommand = new Idle(); // Only create it once
		}
		// start the initial command
		initialCommand.start();
	}

	/**
	 * What the scoring subsystem should do in autonomous
	 * In this case, go to the ball cargo level
	 * In retrospect this should've been the hatch level considering
	 * we start with hatches, but we never got to the point of testing
	 * auto and realizing this lol
	 */
	public void startAuto() {
		(new ArmLevel(ScoringConstants.ScoringLevel.BALL_CARGO)).start();
	}





	@Override
	protected void initDefaultCommand() {
	}

	// we like to limit the current the motors can draw
	// too much current = heat up motors = they burn and could start a fire

	// number of times either motor has exceeded the current limit
	private static int currentLimitCount = 0;

	/**
	 * Test whether the current limit has been exceeded
	 * @return
	 */
	public boolean exceededCurrentLimit()
	{
		// get the current across both motors
		armMotor1Current_amps = armMotor1.getOutputCurrent();
		armMotor2Current_amps = armMotor2.getOutputCurrent();

		// test if either is above the max allowed current
		boolean currentLimit = (armMotor1Current_amps >= ScoringConstants.MAX_ARM_MOTOR_CURRENT_AMPS) ||
							   (armMotor2Current_amps >= ScoringConstants.MAX_ARM_MOTOR_CURRENT_AMPS);
		// if it is, then add to the current limit count because it was exceeded
		if (currentLimit)
		{
			currentLimitCount++;
		}

		// add the number of periodic iterations that the current has exceeded the limit
		// to the dashboard for testing purposes
		//SmartDashboard.putNumber(getName() + "/Arm Current Limit Count", currentLimitCount);

		return currentLimit;
	}

	/*
	 * Front and back limits aren't really necessary
	 * We added them because the arm encoder wasn't functioning properly so
	 * we needed a way to "reset" the position.
	 * When the arm hit the boundary of the robot, we'd know the angle it was
	 * at because it would trigger the limit switch, so we could then set the angle
	 * Then we figured out the problem and didn't need the limit switches, but they are still
	 * nice to have as a safety feature, and kept the angle resetting
	 */

	/*
	 * These functions poll the limit switches for the arm motors and put the result
	 * (true or false depending if the arm is touching the switch) to the Dashboard
	 */

	public boolean frontLimit()
	{
		// get whether the limit switch was triggered
		boolean result = armMotor1.getSensorCollection().isRevLimitSwitchClosed();
		// and put it to the Dashboard for troubleshooting
		SmartDashboard.putBoolean(getName()+"/Arm Front Limit", result);

		// reset the angle because we know where the limit switches are
		if (result) {
			setAngle_deg(ScoringConstants.FRONT_LIMIT_ANGLE);
		}

		return result;
	}
	// same as the front
	public boolean backLimit()
	{
		boolean result = armMotor1.getSensorCollection().isFwdLimitSwitchClosed();
		SmartDashboard.putBoolean(getName()+"/Arm Back Limit", result);

		if (result) {
			setAngle_deg(ScoringConstants.BACK_LIMIT_ANGLE);
		}
		
		return result;
	}

	// finally! the method that does stuff
	@Override
	public void periodic() {
		// put robot in idle if the current is too high to prevent burning motors
		if (exceededCurrentLimit())
		{
			startIdle();
		}

		// Just poll it
		frontLimit();
		backLimit();

		// we had (but never used) a feature called auto align
		// basically the camera would automatically detect the tape and correct the robot's
		// motion so it would line up with the tape and we could put the hatch panel
		// However, the camera is actually blocked by the rest of the arm in the hatch position
		// (very badly designed) so we needed a level for hatch auto aligning, or else the
		// camera would be of no use

		// if drivers requesting auto align, they must start in (teleop) hatch level
		if (oi.autoAlign() && lastLevel == ScoringConstants.ScoringLevel.HP) {
			// go to the auto hatch level to use auto align
			goToLevel(ScoringConstants.ScoringLevel.HP_AUTO);
		}

		// if not auto aligning anymore but were in hatch auto, then just go back to regular auto
		// so drivers can score the hatch
		if (!oi.autoAlign() && lastLevel == ScoringConstants.ScoringLevel.HP_AUTO) {
			goToLevel(ScoringConstants.ScoringLevel.HP);
		}

		// it seems like this bit of code is redundant and checking if we're in manual control twice
		// regardless, it checks that it's in manual control
		if (Math.abs(oi.manualArmControl())>ScoringConstants.ARM_MANUAL_DEADBAND && lastLevel == ScoringConstants.ScoringLevel.MANUAL) {

			// Sets the speed of the arm to a value between -10 and 10.
			double manualArmSpeed = oi.manualArmControl() * ScoringConstants.ARM_MANUAL_SPEED_MAX;

			

			// make manual control relative to a side, not the robot
			// so if going up, the arm will rotate as needed to get closer to the vertical position
			if (lastManualAngle < 0){
				manualArmSpeed = manualArmSpeed * -1;
			}

			// Sets targetAngle to the current arm angle plus the arm speed.
			// Multiplying by the time period causes targetAngle to increment by manualArmSpeed degrees per second.
			double targetAngle = lastManualAngle + (manualArmSpeed * period);
			lastManualAngle = targetAngle;
			targetAngle = Math.abs(targetAngle);

			//SmartDashboard.putNumber(getName()+"/Arm Manual Target Angle (deg)", targetAngle);

			// Tells the arm to move to targetAngle.
			directArmTo(Math.toRadians(targetAngle));

		}

		// note: the OI is set up so that infeed and outfeed WILL BE MUTUALLY EXCLUSIVE
		// and the operator has secondary priority for infeed and outfeed
		boolean infeed = oi.infeedActive();
		boolean outfeed = oi.outfeedActive();
		// we used to use this because we'd use a slower speed for rollers while releasing
		// a hatch but we no longer use rollers for hatches
		boolean hatchOutfeed = (getCommandedLevel() == ScoringConstants.ScoringLevel.HP);
		//SmartDashboard.putBoolean(getName()+"/Infeed", infeed);
		//SmartDashboard.putBoolean(getName()+"/Outfeed", outfeed);

		// set the rollers to 70% in the right direction
		if      (infeed)  { setRollers(-0.7); }
		else if (outfeed) { setRollers(0.7);  }
		else              { setRollers(0.0);  }

		// automatically enable the right camera
		// this already was done but might as well reinforce it
		if (back)
		{
			visionSubsystem.enableBack();
		}
		else
		{
			visionSubsystem.enableFront();
		}

		// whether to grapple (put up) the beak to get a hatch
		boolean grapple = oi.beakGrapple();
		// whether to release the beak to release a hatch
		boolean release = oi.beakRelease();

		// ignore this stuff, we used to use percent output
		// for beak control but burned multiple motors
		// because of it so ugh don't do that

		// if (grapple || release)
		// {
		// 	beakMotor.set(ControlMode.PercentOutput, 0.3);
		// }
		// else
		// {
		// 	beakMotor.set(ControlMode.PercentOutput, 0.0);
		// }

		// again, can't raise beak when arm is in the front or the
		// hatch will fall
		if (!back) {
			// thus if it's in the front, release the beak
// 842
			// but I don't think you could even get to the front
			// with a grappled beak
			beakPosition = BeakPosition.HATCH_RELEASE_BEAK;
		// ^ is the binary XOR operator
		// XOR = exclusive or. it returns 1 if EITHER is true
		// but not BOTH. So if the grapple is true but the beak
		// is also true, a simple or would return true, but we want
		// mutual exclusion
		// sometimes we use binary operations but its not very often
		} else if (grapple ^ release) {
			// if grappling (and not releasing)
			if (grapple) {
				// grapple
				beakPosition = BeakPosition.HATCH_GRAPPLE_BEAK;
			// same logic but for releasing
			} else if (release) {
				beakPosition = BeakPosition.HATCH_RELEASE_BEAK;
			}
		}
		// beakMotor.set(ControlMode.MotionMagic, beakPosition.getBeak_ticks()); // Tell the beak to go to the position set in the enum.
		
		// the beak, having been rushed into the code before a competition, didn't really have the
		// same structure as the rest of the code, which is why the way we control it seems different
		// it does the same but has a different structure

		// check the position of th ebeak
		switch (beakPosition) {
		// if grappling, go to the grappling position
		case HATCH_GRAPPLE_BEAK:
			// these commented out tests were when the motors were burning so we wanted
			// to put in a current limit. I don't think it worked :/
			/*if (beakMotor.getOutputCurrent() >= ScoringConstants.BEAK_MAX_CURRENT_AMPS) {
				beakMotor.set(ControlMode.MotionMagic, beakMotor.getSelectedSensorPosition()); // Tell the beak motor to hold it's current position while in HOLDPOS mode.
				beakPosition = BeakPosition.HATCH_HOLDPOS_BEAK; // Tell the beak motor to enter HOLDPOS mode if the current output exceeds BEAK_MAX_CURRENT_AMPS.
			} else {*/
				beakMotor.set(ControlMode.MotionMagic, beakPosition.getBeak_ticks()); // Tell the beak to go to the position set in the enum.
			//}
			break;

		case HATCH_RELEASE_BEAK:
			beakMotor.set(ControlMode.MotionMagic, beakPosition.getBeak_ticks()); // Tell the beak to go to the position set in the enum.
			break;

		case HATCH_HOLDPOS_BEAK:
			// Do nothing.

		default:
			break;
		}
		
		// just too much telemetry
		clearDiagnosticsEnabled();
		updateBaseDashboard();
		SmartDashboard.putBoolean(getName()+ "/Arm FRONT", !back);
		SmartDashboard.putNumber(getName() + "/Arm Angle", getAngle_deg());
		if (getTelemetryEnabled()) {
			SmartDashboard.putNumber(getName() + "/Arm Ticks", armMotor1.getSelectedSensorPosition());
			SmartDashboard.putNumber(getName() + "/Arm Error", armMotor1.getClosedLoopError());
			SmartDashboard.putNumber(getName() + "/Arm Motor 0 Current", armMotor1Current_amps);
			SmartDashboard.putNumber(getName() + "/Arm Motor 1 Current", armMotor2Current_amps);
			SmartDashboard.putNumber(getName() + "/Arm Motor TOTAL Current", armMotor1Current_amps+armMotor2Current_amps);
			SmartDashboard.putNumber(getName() + "/Beak Ticks", beakMotor.getSelectedSensorPosition());     // Log the beak motor rotation in ticks.
			SmartDashboard.putNumber(getName() + "/Beak Error", beakMotor.getClosedLoopError());            // Log the beak motor error in ticks.
			SmartDashboard.putNumber(getName() + "/Beak Motor Current", beakMotor.getOutputCurrent());      // Log the beak output current in amps.

		}
		// commands will handle dealing with arm manipulation
	}

	@Override
	public void diagnosticsInitialize() {
	}

	@Override
	public void diagnosticsPeriodic() {
		updateBaseDashboard();
		if (getDiagnosticsEnabled())
		{
			double angle = SmartDashboard.getNumber(getName() + "/Test Angle", 0);
			directArmTo(angle);
		}

		// commands will handle dealing with arm manipulation
	}

	@Override
	public void diagnosticsCheck() {		
	}

	@Override
	public void initialize() {
		initializeBaseDashboard();

		//SmartDashboard.putNumber(getName() + "/Test Angle", 0);
	}










	// Physics sim commands



	public TalonSRX getArmMotor1() {
		return armMotor1;
	}
	
	public void manualArmOperate() {
		armMotor1.set(ControlMode.PercentOutput, oi.manualArmRotate());
	}
	public TalonSRX getRollerMotor() {
		return rollerMotor;
	}
}
package frc.robot.subsystem.autonomous;


public class AutonomousConstants {
    public static final double CAMERA_FPS = 30; // TODO: actual

    public static double OFF_AXIS_KP = 2.0;//3.5 (simulator);
	public static final double OFF_AXIS_KI = 0;
	public static final double OFF_AXIS_KD = 0;

	public static double PARALLAX_KP = 0;//0.150 / 100 * 10 * 2;//-150 (simulator);
	public static final double PARALLAX_KI = 0;
    public static final double PARALLAX_KD = 0;
    
    public static final double MIN_PARALLAX_DISTANCE = 0; // (simulator)
	public static final double OFF_AXIS_GAIN_BOOST = 1; // (simulator)

	public static final double GUIDANCE_STOP = 0.1;

	public static final double ASSIST_VELOCITY_IPS = 48.0;
}
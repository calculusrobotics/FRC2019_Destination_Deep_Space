/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.autonomous;

import frc.robot.operatorinterface.OI;
import frc.robot.subsystem.BitBucketSubsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;



public class AutonomousSubsystem extends BitBucketSubsystem {
    private final OI oi = OI.instance();
    private Command initialCommand;
    // Singleton method; use AutonomousSubsystem.instance() to get the AutonomousSubsystem instance.
	public static AutonomousSubsystem instance() {
		if (inst == null) {
            inst = new AutonomousSubsystem();
        }

		return inst;
	}
    private static AutonomousSubsystem inst;
    




    @Override
	protected void initDefaultCommand() {
    }

    @Override
	public void periodic() {
        updateBaseDashboard();
    }



    @Override
	public void diagnosticsInitialize() {
	}

	@Override
	public void diagnosticsPeriodic() {
        updateBaseDashboard();
        if (getDiagnosticsEnabled())
        {

        }

    }

    @Override
	public void diagnosticsCheck() {		
    }
    


    private NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
	private NetworkTable bvTable = networkTable.getTable("BucketVision");
	private NetworkTableEntry bvStateEntry = bvTable.getEntry("BucketVisionState");
    private NetworkTableEntry bvCameraNumber = bvTable.getEntry("CameraNum");
    
    public CameraFeedback getClosestObjectData() {
        int numTargets = (int) bvTable.getEntry("NumTargets").getValue().getDouble();

        double[] distance = bvTable.getEntry("distance").getValue().getDoubleArray();

        double[] pos_x = bvTable.getEntry("pos_x").getValue().getDoubleArray();
        double[] pos_y = bvTable.getEntry("pos_y").getValue().getDoubleArray();

        double[] parallax = bvTable.getEntry("parallax").getValue().getDoubleArray();

        numTargets = Math.min(distance.length,Math.min(pos_x.length,Math.min(pos_y.length,parallax.length))); /// TODO: Temporary

        SmartDashboard.putNumber(getName() + "/Num Targets",numTargets);        
        if (numTargets == 0) {
            return null; // null if no target found (do we want this behavior?)
        }

        int min_index = -1;
        double min_offAxis = 2 * pos_x[0] - 1; // normalize to [-1, 1] from [0, 1]
        for (int i = 0; i < numTargets; i++) {
            double offAxis = 2 * pos_x[i] - 1; // normalize to [-1, 1] from [0, 1]

            if (Math.abs(offAxis) <= Math.abs(min_offAxis)) {
                if ((pos_y[i] > 0.35) && (pos_y[i] < 0.6))
                {
                    min_index = i;
                    min_offAxis = offAxis;
                }
            }
        }

        SmartDashboard.putNumber(getName() + "/Min Index",min_index);

        // If target is not acceptable
        if (min_index == -1)
        {
            return null;
        }

        double offAxis = 2 * pos_x[min_index] - 1; // normalize to [-1, 1] from [0, 1]



        boolean isInAutoAssistRegion = true; // TODO: for now

        return new CameraFeedback(
            isInAutoAssistRegion,
            parallax[min_index],
            offAxis,
            distance[min_index]
        );
    }



    public boolean getUseAutoAssist() {
        return SmartDashboard.getBoolean(getName() + "/AutoAssist", false);
    }


    // Always start the
 
	public void startIdle()
	{
		// Don't use default commands as they can catch you by surprise
		System.out.println("Starting " + getName() + " Idle...");
		if (initialCommand == null)
		{
			initialCommand = new Idle();	// Only create it once
		}
		initialCommand.start();
	}

    @Override
	public void initialize() {
        initializeBaseDashboard();
        
        SmartDashboard.putBoolean(getName() + "/AutoAssist", false);
    }
    


    public void disable() {

    }
}
package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

public class Constants {

    /**This is the time it takes for the PID to update in seconds, 20ms + 110ms sprk max velocity lag  */
    public static final double LOOP_TIME  = 0.13;
    /** The total mass of the robot, in kilograms. Includes Batteries and bumper weight */
    public static final double ROBOT_MASS = Units.lbsToKilograms(85.3);

    public static class DriverConstants{
        public static final double JOYSTICK_DEADBAND = 0.1;
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class SwerveConstants{
        /**The total mass of the drivetrain, eboard, and associated components. Position reflects the aproximate center of mass, relative to the floor, in meters */
        public static final Matter DRIVEBASE = new Matter(new Translation3d(0,0,Units.inchesToMeters(3.5)), ROBOT_MASS);
        /**Time to brake the chassis for after the robot is disabled, in seconds */
        public static final double BRAKE_TIMER_DURATION = 10;
    }
    public static final class Auton
    {
  
      public static final PIDConstants xAutoPID     = new PIDConstants(0.7, 0, 0);
      public static final PIDConstants yAutoPID     = new PIDConstants(0.7, 0, 0);
      public static final PIDConstants angleAutoPID = new PIDConstants(0.4, 0, 0.01);
  
      public static final double MAX_SPEED        = 4;
      public static final double MAX_ACCELERATION = 2;
      public static final PathConstraints constraint = new PathConstraints(MAX_SPEED, MAX_ACCELERATION);

      public static HashMap<String, Command> eventMap = new HashMap<String, Command>() {
        {
            eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        }
    };
    }


    public static class VisionConstants{
        public static final String PhotonCameraName = "FunnyCamera";
        public static final Transform3d CAMERA_POSITION = null;
    }
}

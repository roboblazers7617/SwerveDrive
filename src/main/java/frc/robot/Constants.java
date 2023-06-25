package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

public class Constants {

    /**This is the time it takes for the PID to update in seconds, 20ms + 110ms sprk max velocity lag  */
    public static final double LOOP_TIME  = 0.13;
    /** The total mass of the robot, in kilograms. Includes Batteries and bumper weight */
    public static final double ROBOT_MASS = Units.lbsToKilograms(0); //TODO:Actual Mass

    public static class DriverConstants{
        public static final double joystickDeadband = 0.1;
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class SwerveConstants{
        /**The total mass of the drivetrain, eboard, and associated components. Position reflects the aproximate center of mass, relative to the floor, in meters */
        public static final Matter DRIVEBASE = new Matter(new Translation3d(0,0,0), ROBOT_MASS);//TODO: add actuall position
        /**Time to brake the chassis for after the robot is disabled, in seconds */
        public static final double BRAKE_TIMER_DURATION = 10;
    }
}

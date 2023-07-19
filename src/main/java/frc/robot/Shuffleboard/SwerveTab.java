package frc.robot.Shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Subsystems.Swerve.SwerveDrive;

public class SwerveTab extends ShuffleboardTabBase {

    private final SwerveDrive swerveDrive;
    private final DoublePublisher odometryYPub;
    private final DoublePublisher odometryXPub;
    private final DoublePublisher odometryAnglePub;
    


    public SwerveTab(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        //TODO: "swerveDrive" should be a constant in this class since used more than once
        NetworkTable networkTable = inst.getTable("Shuffleboard/swerveDrive");

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("swerveDrive");

        shuffleboardTab.add(swerveDrive);

        odometryXPub = networkTable.getDoubleTopic("X Odometry").publish();

        odometryYPub = networkTable.getDoubleTopic("Y Odometry").publish();

        odometryAnglePub = networkTable.getDoubleTopic("Angle Odometry").publish();

    }
    @Override
    public void update() {
       odometryAnglePub.set(swerveDrive.getPose().getRotation().getDegrees());
       odometryXPub.set(swerveDrive.getPose().getX());
       odometryYPub.set(swerveDrive.getPose().getY());
        
    }
    
}

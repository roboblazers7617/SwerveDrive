// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shuffleboard;

import java.util.ArrayList;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShuffleboardInfo extends SubsystemBase {
  ArrayList<ShuffleboardTabBase> tabs;

  private static ShuffleboardInfo instance;
  private final BooleanSubscriber topic;

  public static ShuffleboardInfo getInstance() {
    if (instance == null) {
      instance = new ShuffleboardInfo();
    }
    return instance;
  }

  private ShuffleboardInfo() {
    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("debug mode table");
    topic = networkTable.getBooleanTopic("debug mode").subscribe(true);

  }

  /** Creates a new ShuffleboardInfo. */
  public void addTabs(ArrayList<ShuffleboardTabBase> tabs) {
    this.tabs = tabs;

    //TODO: This needs to be documented on what it is doing as it is not clear why this is being done
    for (int i = 0; i < tabs.size(); i ++){
      if(tabs.get(i).toString().contains("DriverStationTab")){
        ShuffleboardTabBase tabHolder = tabs.get(i);
        tabs.remove(i);
        tabs.add(0, tabHolder);
      }
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    //TODO: This needs to be documented on what this logic does as it is not clear
    tabs.get(0).update();
    if (topic.get()) {
      for (int i = 1; i < tabs.size(); i++) {
        tabs.get(i).update();
      }
    }
  }
}

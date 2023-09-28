// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shuffleboard;

import java.util.HashMap;

import frc.robot.Subsystems.Swerve.SwerveDrive;
import shuffleboardlib.Question;
import shuffleboardlib.Questionnaire;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
    private final SwerveDrive swerveDrive;
    public DriverStationTab(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;

        HashMap<String, Question> pieceNumber = new HashMap<>();
        pieceNumber.put("1", new Question("1 piece", null));
        pieceNumber.put("2", new Question("2 piece", null));
        Question pieceNumberQuestion = new Question("How many pieces?", pieceNumber);

        HashMap<String, Question> answersHashMap = new HashMap<>();
        answersHashMap.put("left", pieceNumberQuestion);
        answersHashMap.put("middle", new Question("Do you like sushi?", null));
        answersHashMap.put("right", pieceNumberQuestion);

        new Questionnaire("Driver Station", new Question("starting position", answersHashMap), 3);

    }
    
    @Override
    public void update() {
    }
}

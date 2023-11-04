// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shuffleboard;

import java.util.HashMap;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import shuffleboardlib.Question;
import shuffleboardlib.Questionnaire;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
    private final SwerveDrive swerveDrive;
    private Questionnaire questionnaire;
    private final StringLogEntry logger;


    public DriverStationTab(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        
        HashMap<String, Question> leftPieceNumber = new HashMap<>();
        leftPieceNumber.put("1", new Question("1 piece", null, true, "leftOne"));
        leftPieceNumber.put("2", new Question("2 piece", null, true, "leftTwo"));
        Question leftPieceNumberQuestion = new Question("How many pieces?", leftPieceNumber, false, null);

        HashMap<String, Question> rightPieceNumber = new HashMap<>();
        rightPieceNumber.put("1", new Question("1 piece", null, true, "rightOne"));
        rightPieceNumber.put("2", new Question("2 piece", null, true, "rightTwo"));
        Question rightPieceNumberQuestion = new Question("How many pieces?", leftPieceNumber, false, null);

        HashMap<String, Question> answersHashMap = new HashMap<>();
        answersHashMap.put("left", leftPieceNumberQuestion);
        answersHashMap.put("middle", new Question ("middle", null, true, "middleCone"));
        answersHashMap.put("right", rightPieceNumberQuestion);
        Question rootQuestion = new Question("starting position", answersHashMap, false, null);

        questionnaire = new Questionnaire("Driver Station", rootQuestion, 5);

        //create data logger
        logger = new StringLogEntry(DataLogManager.getLog(), "/auto");
    }

    @Override
    public void update() {
        logger.append(questionnaire.getOutput());
    }
}

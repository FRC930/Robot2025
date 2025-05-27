package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;

public class ReefPositionsUtil {

    public static enum ScoreLevel {
        L1,
        L2,
        L3,
        L4
    } 

    public static enum DeAlgaeLevel {
        Top,
        Low
    }

    public static enum AutoAlignSide {
        Left,
        Right
    }
    
    // Defaults
    private ScoreLevel selectedScoreLevel = ScoreLevel.L1;
    private DeAlgaeLevel selectedDeAlgaeLevel = DeAlgaeLevel.Low;
    private AutoAlignSide selectedAutoAlignSide = AutoAlignSide.Left;

    private boolean isAutoAligning = true;

    private static ReefPositionsUtil instance;

    public static ReefPositionsUtil getInstance() {
        if (instance == null) {
            instance = new ReefPositionsUtil();
        }
        return instance;
    }

    private ReefPositionsUtil() {
        selectedScoreLevel = ScoreLevel.L1;
        selectedDeAlgaeLevel = DeAlgaeLevel.Low;
        selectedAutoAlignSide = AutoAlignSide.Left;
    }

    /**
     * Sets selected level variable to given ScoreLevel value. 
     * For a command that runs this method use getNewSetScoreLevelCommand(ScoreLevel)
     * 
     * @param level the desired level to select (L1 is Trough)
     */
    public void setScoreLevel(ScoreLevel level) {
        selectedScoreLevel = level;
    }

    /**
     * Creates a new command that sets the selected level variable. 
     * For the runnable itself use setScoreLevel(ScoreLevel)
     * 
     * @param level the desired level to select (L1 is Trough)
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetScoreLevelCommand(ScoreLevel level) {
        return new InstantCommand(() -> setScoreLevel(level));
    }

    /**
     * Use to determine which reef score level is currently selected. Useful for logging.
     * For a boolean output, use isSelected(ScoreLevel level)
     * 
     * @return the currently selected scoring level
     */
    public ScoreLevel getScoreLevel() {
        return selectedScoreLevel;
    }

    /**
     * Use to determine whether selected position is the given level. Useful for conditional commands. 
     * For simply determining which is selected, use getScoreLevel()
     * 
     * Input a ScoreLevel to check the scoring level, and DeAlgaeLevel to check the dealgaefy level
     * 
     * @param level the score level to check
     * @return whether the selected scoring level is the same as the <b>level</b> parameter
     */
    public boolean isSelected(ScoreLevel level) {
        return (level.equals(selectedScoreLevel));
    }

    /**
     * Sets selected level variable to given DeAlgae value. 
     * For a command that runs this method use getNewSetDeAlgaeLevelCommand(ScoreLevel)
     * 
     * @param level the desired level to select (Top is between L3 and L4; Bottom is between L2 and L3)
     */
    public void setDeAlgaeLevel(DeAlgaeLevel level) {
        selectedDeAlgaeLevel = level;
    }

    /**
     * Sets selected level variable to given DeAlgae value. 
     * For a command that runs this method use getNewSetDeAlgaeLevelCommand(ScoreLevel)
     * 
     * @param level the desired level to select (Top is between L3 and L4; Bottom is between L2 and L3)
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetDeAlgaeLevelCommand(DeAlgaeLevel level) {
        return new InstantCommand(() -> setDeAlgaeLevel(level));
    }

    /**
     * Use to determine which dealgae level is currently selected. Useful for logging.
     * For a boolean output, use isSelected(DeAlgaeLevel level)
     * 
     * @return the currently selected dealgaefy level
     */
    public DeAlgaeLevel getDeAlgaeLevel() {
        return selectedDeAlgaeLevel;
    }

    /**
     * Use to determine whether selected position is the given level. Useful for conditional commands. 
     * For simply determining which is selected, use getDeAlgaeLevel()
     * 
     * Input a ScoreLevel to check the scoring level, and DeAlgaeLevel to check the dealgaefy level
     * 
     * @param level the dealgae level to check
     * @return whether the selected dealgaefy level is the same as the <b>level</b> parameter
     */
    public boolean isSelected(DeAlgaeLevel level) {
        return (level.equals(selectedDeAlgaeLevel));
    }

    /**
     * Sets selected level variable to given AutoAlignSide value. 
     * For a command that runs this method use getNewSetAutoAlignLevelCommand(AutoAlignLevel)
     * 
     */
    public void setAutoAlignSide(AutoAlignSide level) {
        selectedAutoAlignSide = level;
    }

    /**
     * Creates a new command that sets the selected side variable. 
     * For the runnable itself use setAutoAlignSide(AutoAlignSide)
     * 
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetAutoAlignCommand(AutoAlignSide side) {
        return new InstantCommand(() -> setAutoAlignSide(side));
    }

    /**
     * Use to determine which auto align side is currently selected. Useful for logging.
     * For a boolean output, use isSelected(AutoAlignLevel level)
     * 
     * @return the currently selected auto align side
     */
    public AutoAlignSide getAutoAlignSide() {
        return selectedAutoAlignSide;
    }

    public boolean isSelected(AutoAlignSide side) {
        return (side.equals(selectedAutoAlignSide));
    }
    
    public void setIsAutoAligning(boolean isAutoAligning) {
        this.isAutoAligning = isAutoAligning;
    }

    public boolean getIsAutoAligning() {
        return isAutoAligning;
    }

    /**
     * Use to determine whether selected position is the given level. Useful for conditional commands. 
     * For simply determining which is selected, use getAutoAlignLevel()
     * 
     * Input a AutoAlignSide to check the auto align level.
     * 
     * @param level the auto align level to check
     * @return whether the selected auto align level is the same as the <b>level</b> parameter
     */
    public Command getCoralLevelSelector(Map<ScoreLevel,Command> bindings) {
        return new SelectCommand<>(bindings, this::getScoreLevel);
    }
}

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoringModesUtil {

    public static enum CoralLevel {
        L1,
        L2,
        L3,
        L4
    } 

    public static enum DeAlgaeLevel {
        Top,
        Low
    }

    public static enum ScoringMode {
        Coral,
        Net,
        Processor,
        Stow
    }

    // Defaults
    private CoralLevel selectedCoralLevel = CoralLevel.L1;
    private DeAlgaeLevel selectedDeAlgaeLevel = DeAlgaeLevel.Low;
    private ScoringMode selectedScoringMode = ScoringMode.Stow;

    private static ScoringModesUtil instance;

    public static ScoringModesUtil getInstance() {
        if (instance == null) {
            instance = new ScoringModesUtil();
        }
        return instance;
    }

    /**
     * Sets selected level variable to given CoralLevel value. 
     * <p>For a command that runs this method use getNewSetCoralLevelCommand(CoralLevel)</p>
     * 
     * @param level the desired level to select (L1 is Trough)
     */
    public void setCoralLevel(CoralLevel level) {
        selectedCoralLevel = level;
    }

    /**
     * Creates a new command that sets the selected level variable. 
     * <p>For the runnable itself use setCoralLevel(CoralLevel)</p>
     * 
     * @param level the desired level to select (L1 is Trough)
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetCoralLevelCommand(CoralLevel level) {
        return new InstantCommand(() -> setCoralLevel(level));
    }

    /**
     * Use to determine which reef score level is currently selected. Useful for logging.
     * <p>For a boolean output, use isSelected(CoralLevel level)</p>
     * 
     * @return the currently selected scoring level
     */
    public CoralLevel getCoralLevel() {
        return selectedCoralLevel;
    }

    /**
     * Use to determine whether selected position is the given level. Useful for conditional commands. 
     * For simply determining which is selected, use getCoralLevel().
     * 
     * <p>Input a CoralLevel to check the coral reef score level, a DeAlgaeLevel to check the dealgaefy level, and a ScoringMode to check the scoring mode</p>
     * 
     * @param level the score level to check
     * @return whether the selected coral level is the same as the <b>level</b> parameter
     */
    public boolean isSelected(CoralLevel level) {
        return (level.equals(selectedCoralLevel));
    }

    /**
     * Sets selected level variable to given DeAlgae value. 
     * <p>For a command that runs this method use getNewSetDeAlgaeLevelCommand(ScoreLevel)</p>
     * 
     * @param level the desired level to select (Top is between L3 and L4; Bottom is between L2 and L3)
     */
    public void setDeAlgaeLevel(DeAlgaeLevel level) {
        selectedDeAlgaeLevel = level;
    }

    /**
     * Sets selected level variable to given DeAlgae value. 
     * <p>For a command that runs this method use getNewSetDeAlgaeLevelCommand(ScoreLevel)</p>
     * 
     * @param level the desired level to select (Top is between L3 and L4; Bottom is between L2 and L3)
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetDeAlgaeLevel(DeAlgaeLevel level) {
        return new InstantCommand(() -> setDeAlgaeLevel(level));
    }

    /**
     * Use to determine which dealgae level is currently selected. Useful for logging.
     * <p>For a boolean output, use isSelected(DeAlgaeLevel level)</p>
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
     * <p>Input a CoralLevel to check the coral reef score level, a DeAlgaeLevel to check the dealgaefy level, and a ScoringMode to check the scoring mode</p>
     * 
     * @param level the dealgae level to check
     * @return whether the selected dealgaefy level is the same as the <b>level</b> parameter
     */
    public boolean isSelected(DeAlgaeLevel level) {
        return (level.equals(selectedDeAlgaeLevel));
    }

    /**
     * Sets selected mode variable to given ScoringMode value. 
     * <p>For a command that runs this method use getNewSetScoringModeCommand(ScoringMode)</p>
     * 
     * @param mode the desired mode to select
     */
    public void setScoringMode(ScoringMode mode) {
        selectedScoringMode = mode;
    }

    /**
     * Creates a new command that sets the selected mode variable. 
     * <p>For the runnable itself use setScoringMode(ScoringMode)</p>
     * 
     * @param mode the desired mode to select
     * @return an instant command that runs the set method
     */
    public InstantCommand getNewSetScoringModeCommand(ScoringMode mode) {
        return new InstantCommand(() -> setScoringMode(mode));
    }

    /**
     * Use to determine which scoring mode is currently selected. Useful for logging.
     * <p>For a boolean output, use isSelected(ScoringMode mode)</p>
     * 
     * @return the currently selected scoring mode
     */
    public ScoringMode getScoringMode() {
        return selectedScoringMode;
    }

    /**
     * Use to determine whether selected scoring mode is the given mode. Useful for conditional commands. 
     * For simply determining which is selected, use getScoringMode().
     * 
     * <p>Input a CoralLevel to check the coral reef score level, a DeAlgaeLevel to check the dealgaefy level, and a ScoringMode to check the scoring mode</p>
     * 
     * @param mode the scoring type to check
     * @return whether the selected scoring mode is the same as the <b>mode</b> parameter
     */
    public boolean isSelected(ScoringMode mode) {
        return (mode.equals(selectedScoringMode));
    }
}

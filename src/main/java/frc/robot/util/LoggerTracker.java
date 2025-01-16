package frc.robot.util;

import java.util.HashMap;
import java.util.function.Consumer;

/** 
 *  Class with a map of logs and runnables to run when logs are changed. 
 * */
public class LoggerTracker {
  HashMap<LoggedTunableNumber, Consumer<LoggedTunableNumber>> logs;

  /** Create a blank LoggerTracker */
  public LoggerTracker() {
    logs = new HashMap<LoggedTunableNumber, Consumer<LoggedTunableNumber>>();
  }

  /**
   * Create a LoggerTracker with an initial HashMap
   *
   * @param initial The initial HashMap for the LoggerTracker.
   */
  public LoggerTracker(HashMap<LoggedTunableNumber, Consumer<LoggedTunableNumber>> initial) {
    logs = initial;
  }

  /**
   * Add a log to the tracker
   *
   * @param log The LoggedTunableNumber to add
   * @param toRun The Runnable that will run if changed
   */
  public void addLog(LoggedTunableNumber log, Consumer<LoggedTunableNumber> toRun) {
    if (logs.containsKey(log) && logs.containsValue(toRun)) {
      // TODO: Handle if already in map
      return;
    }
    logs.put(log, toRun);
  }

  /**
   * Remove the Runnables associated to a log
   *
   * @param key The log to obliterate
   */
  public void removeLog(LoggedTunableNumber key) {
    logs.remove(key);
  }

  /*
   * Run all changed logs' Runnables
   */
  public void periodic() {
    logs.forEach(
        (log, toRun) -> {
          if (log.hasChanged(this.hashCode())) {
            toRun.accept(log);
          }
        });
  }
}

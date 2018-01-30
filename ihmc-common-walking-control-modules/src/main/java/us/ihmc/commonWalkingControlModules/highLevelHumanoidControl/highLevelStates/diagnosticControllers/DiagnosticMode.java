package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.diagnosticControllers;

/**
 * Provides a simple and human-readable list of the diagnostics that can be ran on the robot.
 */
public enum DiagnosticMode
{
   /**
    * {@code MANUAL} is typically associated with {@link ManualDiagnosticController} which allows
    * the user to focus and debug each joint of the robot.
    */
   MANUAL;
}

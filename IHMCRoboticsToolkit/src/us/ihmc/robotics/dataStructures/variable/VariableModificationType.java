package us.ihmc.robotics.dataStructures.variable;

public enum VariableModificationType
{
   /** A TUNABLE yoVariable can be modified by the user from the SCS GUI */
   TUNABLE,
   /** A STATE yoVariable should not be modified from the SCS GUI it's purpose is rewindability */
   STATE,
   /** A DEBUG yoVariable should not be modified from the SCS GUI it's purpose is visualization */
   DEBUG
}

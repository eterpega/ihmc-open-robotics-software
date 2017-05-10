package us.ihmc.robotics.dataStructures.variable;

import java.awt.Color;

public enum VariableModificationType
{
   /** A TUNABLE yoVariable can be modified by the user from the SCS GUI */
   TUNABLE,
   /** A STATE yoVariable should not be modified from the SCS GUI it's purpose is rewindability */
   STATE,
   /** A DEBUG yoVariable should not be modified from the SCS GUI it's purpose is visualization */
   DEBUG;

   public boolean getIsModifiable()
   {
      switch (this)
      {
      case TUNABLE:
         return true;
      default:
         return false;
      }
   }

   public Color getTextColor()
   {
      switch (this)
      {
      case TUNABLE:
         return Color.GREEN;
      case DEBUG:
         return Color.RED;
      case STATE:
      default:
         return Color.BLACK;
      }
   }
}

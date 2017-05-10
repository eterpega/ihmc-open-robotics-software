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
         return Color.BLUE;
      case DEBUG:
         return Color.MAGENTA;
      case STATE:
      default:
         return Color.BLACK;
      }
   }

   public boolean isEnabledByDefault()
   {
      switch (this)
      {
      case TUNABLE:
         return true;
      case DEBUG:
         return true;
      case STATE:
         return false;
      default:
         return false;
      }
   }
}

package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class PinJointDescription extends OneDoFJointDescription
{

   public static PinJointDescriptionBuilder builder()
   {
      return new PinJointDescriptionBuilder();
   }
}

package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class SliderJointDescription extends OneDoFJointDescription
{
   public static SliderJointDescriptionBuilder builder()
   {
      return new SliderJointDescriptionBuilder();
   }

   static abstract class Builder implements JointDescription.Builder {}
}

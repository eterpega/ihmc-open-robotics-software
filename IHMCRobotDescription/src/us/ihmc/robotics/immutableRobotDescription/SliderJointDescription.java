package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class SliderJointDescription extends OneDoFJointDescription
{
   public static ImmutableSliderJointDescription.Builder builder()
   {
      return ImmutableSliderJointDescription.builder();
   }

   static abstract class Builder implements OneDoFJointDescription.Builder
   {
   }
}

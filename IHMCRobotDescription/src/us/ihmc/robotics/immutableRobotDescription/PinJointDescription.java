package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;

@Immutable public abstract class PinJointDescription extends OneDoFJointDescription
{

   public static ImmutablePinJointDescription.Builder builder()
   {
      return ImmutablePinJointDescription.builder();
   }

   static abstract class Builder implements OneDoFJointDescription.Builder
   {
   }
}

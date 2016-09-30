package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class PinJointDescription extends OneDoFJointDescription
{

   @Override public ModifiablePinJointDescription toModifiable()
   {
      return ModifiablePinJointDescription.create().from(this);
   }

   public static ImmutablePinJointDescription.Builder builder()
   {
      return ImmutablePinJointDescription.builder();
   }

   static abstract class Builder implements OneDoFJointDescription.Builder
   {
   }
}

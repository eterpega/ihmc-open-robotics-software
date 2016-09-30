package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class SliderJointDescription extends OneDoFJointDescription
{
   @Override public ModifiableSliderJointDescription toModifiable()
   {
      return ModifiableSliderJointDescription.create().from(this);
   }

   public static ImmutableSliderJointDescription.Builder builder()
   {
      return ImmutableSliderJointDescription.builder();
   }

   static abstract class Builder implements OneDoFJointDescription.Builder
   {
   }
}

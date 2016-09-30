package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;

@Immutable @Modifiable public abstract class JointWrenchSensorDescription extends SensorDescription
{
   @Override public ModifiableJointWrenchSensorDescription toModifiable()
   {
      return ModifiableJointWrenchSensorDescription.create().from(this);
   }

   public static ImmutableJointWrenchSensorDescription.Builder builder()
   {
      return ImmutableJointWrenchSensorDescription.builder();
   }
}

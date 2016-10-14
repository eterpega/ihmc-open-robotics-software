package us.ihmc.robotics.immutableRobotDescription;

import org.immutables.value.Value.Default;
import org.immutables.value.Value.Immutable;
import org.immutables.value.Value.Modifiable;
import us.ihmc.robotics.immutableRobotDescription.graphics.GraphicsGroupDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.ShapeDescription;

import javax.vecmath.Vector3d;
import java.util.Collection;

@Immutable @Modifiable public abstract class RobotDescription extends JointDescription implements GraphicsObjectsHolder
{
   @Default @Override public Vector3d getOffsetFromJoint()
   {
      return new Vector3d();
   }

   @Default @Override public LinkDescription getLink()
   {
      return LinkDescription.empty("empty_" + getName());
   }

   @Override public ModifiableRobotDescription toModifiable()
   {
      return ModifiableRobotDescription.create().from(this);
   }

   // TODO: the following methods need to be tested
   public JointDescription getJointDescription(String name)
   {
      for (JointDescription rootJoint : getChildrenJoints())
      {
         JointDescription jointDescription = getJointDescriptionRecursively(name, rootJoint);
         if (jointDescription != null)
            return jointDescription;
      }

      return null;
   }

   private JointDescription getJointDescriptionRecursively(String name, JointDescription jointDescription)
   {
      if (jointDescription.getName().equals(name))
         return jointDescription;

      Collection<JointDescription> childJointDescriptions = jointDescription.getChildrenJoints();
      for (JointDescription childJointDescription : childJointDescriptions)
      {
         JointDescription jointDescriptionRecursively = getJointDescriptionRecursively(name, childJointDescription);
         if (jointDescriptionRecursively != null)
            return jointDescriptionRecursively;
      }
      return null;
   }

   @Override public GraphicsGroupDescription getCollisionObject(String name)
   {
      //TODO: Fix up for collision meshes to work...
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getLinkGraphics();
   }

   @Override public GraphicsGroupDescription getGraphicsObject(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getLinkGraphics();
   }

   @Override public String toString()
   {
      return super.toString();
   }

   public static ImmutableRobotDescription.Builder builder()
   {
      return ImmutableRobotDescription.builder();
   }

   static abstract class Builder implements JointDescription.Builder
   {
   }
}

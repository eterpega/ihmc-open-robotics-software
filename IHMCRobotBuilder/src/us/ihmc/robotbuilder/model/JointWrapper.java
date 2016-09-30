package us.ihmc.robotbuilder.model;

import us.ihmc.robotics.immutableRobotDescription.JointDescription;

/**
 *
 */
public class JointWrapper
{
   private final JointDescription jointDescription;

   public JointWrapper(JointDescription jointDescription)
   {
      this.jointDescription = jointDescription;
   }

   public JointDescription getJointDescription()
   {
      return jointDescription;
   }

   @Override public boolean equals(Object o)
   {
      if (this == o)
         return true;
      if (o == null || getClass() != o.getClass())
         return false;

      JointWrapper that = (JointWrapper) o;

      return jointDescription.equals(that.jointDescription);

   }

   @Override public int hashCode()
   {
      return jointDescription.hashCode();
   }

   @Override public String toString()
   {
      return jointDescription.getName();
   }
}

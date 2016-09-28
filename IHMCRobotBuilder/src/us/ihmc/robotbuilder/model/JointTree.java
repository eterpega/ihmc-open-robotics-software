package us.ihmc.robotbuilder.model;

import javaslang.collection.List;
import us.ihmc.robotbuilder.util.Tree;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;

/**
 *
 */
public class JointTree implements Tree<JointTree>
{
   private final JointDescription jointDescription;
   private final boolean expanded;
   //private final List<JointDescription> children;

   public JointTree(JointDescription jointDescription, boolean expanded)
   {
      this.jointDescription = jointDescription;
      this.expanded = expanded;
      //children = List.ofAll(jointDescription.getChildrenJoints()).map(joint -> new JointTree(joint, false));
   }

   @Override public Iterable<JointTree> getChildren()
   {
      return null;
   }
}

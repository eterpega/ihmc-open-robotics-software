package us.ihmc.robotics.immutableRobotDescription;

import us.ihmc.robotics.immutableRobotDescription.graphics.GraphicsGroupDescription;
import us.ihmc.robotics.util.Tree;

public class RobotDescription implements GraphicsObjectsHolder, NamedObject
{
   private final Tree<JointDescription> rootJoint;
   private final String name;

   public RobotDescription(Tree<JointDescription> rootJoint, String name)
   {
      this.rootJoint = rootJoint;
      this.name = name;
   }

   public Tree<JointDescription> getRootJoint()
   {
      return rootJoint;
   }

   public RobotDescription withRootJoint(Tree<JointDescription> newRoot)
   {
      return new RobotDescription(newRoot, name);
   }

   // TODO: the following methods need to be tested
   public JointDescription getJointDescription(String name)
   {
      return rootJoint.stream()
                      .filter(node -> name.equals(node.getName()))
                      .findFirst()
                      .orElse(null);
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
      return rootJoint.toString();
   }

   @Override public String getName()
   {
      return name;
   }

   public RobotDescription withName(String newName)
   {
      return new RobotDescription(rootJoint, newName);
   }
}

package us.ihmc.robotbuilder.gui;

import org.jetbrains.annotations.NotNull;
import us.ihmc.robotics.robotDescription.JointDescription;

import java.util.Optional;

public class JointDropHolder
{
   private static JointDropHolder jointDropHolder = new JointDropHolder();

   private Class<? extends JointDescription> joint;

   public static JointDropHolder getInstance() {
      return jointDropHolder;
   }

   public JointDropHolder setJoint(@NotNull Class<? extends JointDescription> joint) {
      System.out.println("Set joint to "+joint.getName());
      this.joint = joint;
      return this;
   }

   public Optional<Class<? extends JointDescription>> getJoint() {
      return Optional.of(this.joint);
   }

   public void clearJoint() {
      System.out.println("cleared");
      this.joint = null;
   }
}

package us.ihmc.robotbuilder.gui;

import us.ihmc.robotics.robotDescription.*;

import java.util.HashMap;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class JointDescriptionLabels
{
   private static HashMap<String, Class<? extends JointDescription>> labeledJoints = new HashMap<>();

   private static void initialize() {
      labeledJoints.putAll(
            Stream.of(
                  PinJointDescription.class,
                  SliderJointDescription.class
            )
               .collect(
                     Collectors.toMap(
                        joint -> joint.getSimpleName().replace("Description",""),
                        joint -> joint
                     )
               )
      );
   }

   public static HashMap<String, Class<? extends JointDescription>> getAllLabeledJoints() {
      if (labeledJoints.isEmpty()) {
         initialize();
      }

      return labeledJoints;
   }

   public static Optional<Class<? extends JointDescription>> getJointDescriptionFromLabel(String label) {
      return Optional.of(labeledJoints.get(label));
   }
}

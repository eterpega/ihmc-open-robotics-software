package us.ihmc.manipulation.planning.collisionAvoidance;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotDescription.CapsuleDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ConvexPolytopeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.ConvexShapeDescription;
import us.ihmc.robotics.robotDescription.CubeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CylinderDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RobotCollisionMeshProvider
{
   public RobotCollisionMeshProvider()
   {

   }

   public void createCollisionMeshesFromRobotDescription(FullRobotModel robotModel, RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJoints = robotDescription.getRootJoints();
      if (rootJoints.size() > 1 || !(rootJoints.get(0) instanceof FloatingJointDescription))
         throw new RuntimeException("There should be only one floating joint");
   }

   public FrameConvexPolytope createCollisionMesh(RigidBody rigidBody, CollisionMeshDescription meshDescription)
   {
      ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
      FrameConvexPolytope collisionMesh = new FrameConvexPolytope(bodyFixedFrame);
      
      ArrayList<ConvexShapeDescription> collisionShapeDescriptions = new ArrayList<>();
      meshDescription.getConvexShapeDescriptions(collisionShapeDescriptions);
      for (ConvexShapeDescription shapeDescription : collisionShapeDescriptions)
      {
         if (shapeDescription instanceof SphereDescriptionReadOnly)
         {
            
         }
         else if (shapeDescription instanceof CapsuleDescriptionReadOnly)
         {

         }
         else if (shapeDescription instanceof CylinderDescriptionReadOnly)
         {

         }
         else if (shapeDescription instanceof CubeDescriptionReadOnly)
         {

         }
         else if (shapeDescription instanceof ConvexPolytopeDescriptionReadOnly)
         {

         }
         else
            throw new RuntimeException("Unhandled collision mesh description shape: " + shapeDescription.getClass()); 

      }
      return collisionMesh;
   }
}

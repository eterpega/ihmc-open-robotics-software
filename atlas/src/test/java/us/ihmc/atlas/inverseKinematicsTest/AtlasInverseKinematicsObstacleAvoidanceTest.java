package us.ihmc.atlas.inverseKinematicsTest;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import com.badlogic.gdx.math.collision.BoundingBox;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarInverseKinematicsObstacleAvoidanceTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotDescription.ConvexShapeDescription;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;

public class AtlasInverseKinematicsObstacleAvoidanceTest extends AvatarInverseKinematicsObstacleAvoidanceTest
{
   AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

   @Override
   public boolean keepSCSUp()
   {
      return true;
   }

   @Override
   public int getNumberOfObstacles()
   {
      return 3;
   }

   @Override
   public boolean specifyObstacles()
   {
      return true;
   }

   @Override
   public BoundingBox getWorkspaceBoundsForCreatingObstacles()
   {
      return null;
   }

   @Override
   public List<? extends ConvexShapeDescription> getObstacleDescription()
   {
      List<ConvexShapeDescription> obstacleDescriptionList = new ArrayList<ConvexShapeDescription>();
      SphereDescriptionReadOnly sphere = new SphereDescriptionReadOnly(0.1, new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.4, -0.25, 0.4)));
      obstacleDescriptionList.add(sphere);
      //sphere = new SphereDescriptionReadOnly(0.1, new RigidBodyTransform(new RotationMatrix(), new Vector3D(-0.27, 0.0, 0.5)));
      sphere = new SphereDescriptionReadOnly(0.1, new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.4, 0.25, 0.4)));
      obstacleDescriptionList.add(sphere);
      sphere = new SphereDescriptionReadOnly(0.1, new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.0, -0.45, -0.4)));
      obstacleDescriptionList.add(sphere);
      return obstacleDescriptionList;
   }

   @Override
   public Point3D getDesiredHandLocation()
   {
      return new Point3D(0.5, -0.1, 0.0);
   }

   @Test
   @Override
   public void testInverseKinematics()
   {
      super.testInverseKinematics();
   }

   @ContinuousIntegrationTest(estimatedDuration = 25.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test //(timeout = 300000)
   public void testRandomHandPositionsInverseKinematics()
   {
      super.testRandomHandPositionsInverseKinematics();
   }
   
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }
}

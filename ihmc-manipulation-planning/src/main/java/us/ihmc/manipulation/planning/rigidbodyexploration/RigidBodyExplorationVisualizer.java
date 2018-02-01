package us.ihmc.manipulation.planning.rigidbodyexploration;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ReachingManifoldMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RigidBodyExplorationVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(46561L);

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryDuration = 5.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryDuration / dt / recordFrequency + 2);;

   public RigidBodyExplorationVisualizer()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();

      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);

      Robot robot = new Robot("dummy");
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setGroundVisible(false);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      // Initial, Manifold
      FramePose3D initialRigidBody = new FramePose3D(worldFrame);

      RigidBody dummyHand = new RigidBody("dummyHand", worldFrame);
      Point3D manifoldOrigin = new Point3D(1.0, 1.0, 1.0);
      Quaternion manifoldOrientation = new Quaternion();
      ReachingManifoldMessage manifoldMessage = WholeBodyTrajectoryToolboxMessageTools.createSphereManifoldMessages(dummyHand, manifoldOrigin, 0.1);

      scs.addStaticLinkGraphics(createManifoldMessageVisualization(manifoldMessage, 0.01, YoAppearance.AliceBlue()));

      // SpatialDataRegion
      SpatialDataRegion spatialDataRegion = new SpatialDataRegion();
      spatialDataRegion.setPositionLowerLimitSpace(-1.0, -1.0, -1.0);
      spatialDataRegion.setPositionUpperLimitSpace(1, 1, 1);
//      spatialDataRegion.setPositionLowerLimitSpace(-0.5, -0.5, -0.5);
//      spatialDataRegion.setPositionUpperLimitSpace(1, 1, 1);

      spatialDataRegion.setRotationVectorLowerLimitSpace(-1, -1, -1);
      spatialDataRegion.setRotationVectorUpperLimitSpace(1, 1, 1);

      // Initial ExploringRigidBody
      Pose3D initialSpatialData = new Pose3D();
      initialSpatialData.appendTranslation(0.0, 0.0, 0.0);
      ExploringRigidBody initialExploringRigidBody = new ExploringRigidBody(initialSpatialData);
      scs.addStaticLinkGraphics(createExploringRigidBodyVisualization(initialExploringRigidBody));

      //      // ********************************************** //
      //      // adaptive random region test
      //      // ********************************************** //
      //      int numberOfExpansion = 1000;
      //      double a = 0.01;
      //      double b = 0.3;
      //      double c = 1.0;
      //
      //      double ak = a;
      //      double bk = b;
      //      double ck = c;
      //
      //      double max = 0.0;
      //      if (a > max)
      //         max = a;
      //      if (b > max)
      //         max = b;
      //      if (c > max)
      //         max = c;
      //      double expandingLimitRatio = 1.0;
      //
      //      double expandingLimit = max * (1 + expandingLimitRatio);
      //
      //      double ampA = expandingLimit - a;
      //      double ampB = expandingLimit - b;
      //      double ampC = expandingLimit - c;
      //
      //      for (int i = 0; i < numberOfExpansion; i++)
      //      {
      //         double alpha = (double) ((double) i / (double) numberOfExpansion) * Math.PI;
      //         ak = a + 0.5 * ampA * (1 - Math.cos(alpha));
      //         bk = b + 0.5 * ampB * (1 - Math.cos(alpha));
      //         ck = c + 0.5 * ampC * (1 - Math.cos(alpha));
      //      }
      //      // ********************************************** //      

      // Random exploring      
      int numberOfExploring = 500;
      double stepSize = 0.1;
      ArrayList<ExploringRigidBody> exploringRigidBodies = new ArrayList<>();
      exploringRigidBodies.add(initialExploringRigidBody);

      for (int i = 0; i < numberOfExploring; i++)
      {
         // random spatial data.
         Pose3D randomSpatialData = new Pose3D();
         spatialDataRegion.getRandomSpatialData(randomSpatialData);

         // find closest one.
         double minimumDistance = Double.MAX_VALUE;
         int minimumIndex = 0;
         for (int j = 0; j < exploringRigidBodies.size(); j++)
         {
            double distance = exploringRigidBodies.get(j).getDistance(randomSpatialData);
            if (minimumDistance > distance)
            {
               minimumIndex = j;
               minimumDistance = distance;
            }
         }

         // create new expanding one.
         double alpha = stepSize / minimumDistance;
         ExploringRigidBody candidateParentRigidBody = exploringRigidBodies.get(minimumIndex);
         Pose3D newSpatialData = new Pose3D(candidateParentRigidBody.getSpatialData());
         newSpatialData.interpolate(randomSpatialData, alpha);

         // test validity
         boolean validity = true;
         Point3D originOfObstacle = new Point3D(-0.1, 0.3, -0.1);
         double distanceWObstacle = originOfObstacle.distance(newSpatialData.getPosition());
         if(distanceWObstacle < 0.3)
            validity = false;
         
         
         
         ExploringRigidBody exploringRigidBody = new ExploringRigidBody();
         if (validity)
         {
            exploringRigidBody.setSpatialData(newSpatialData);
            exploringRigidBody.setParentRigidBody(candidateParentRigidBody);

            // add it.
            exploringRigidBodies.add(exploringRigidBody);

            // visualize it.
            scs.addStaticLinkGraphics(createExploringRigidBodyVisualization(exploringRigidBody));
            scs.addStaticLinkGraphics(createConnectionVisualization(candidateParentRigidBody, exploringRigidBody));

         }
         else
         {
            scs.addStaticLinkGraphics(createExploringRigidBodyVisualization(exploringRigidBody));
            continue;
         }
      }
      
      PrintTools.info("numberOfTrialsToExpand is "+numberOfExploring);
      PrintTools.info("numberOfValidExpanding is "+exploringRigidBodies.size());
      
      
      
      // rotation vector distance test
      Vector3D rv1 = new Vector3D(0.5, 0.5, 0.5);
      Vector3D rv2 = new Vector3D(1.0, 1.0, 1.0);
      Vector3D rv3 = new Vector3D(1.0, 0.0, 1.0);
      
      Quaternion qt1 = new Quaternion(rv1);
      Quaternion qt2 = new Quaternion(rv2);
      Quaternion qt3 = new Quaternion(rv3);
      
      PrintTools.info(""+qt1.distance(qt2));
      PrintTools.info(""+qt1.distance(qt3));
      PrintTools.info(""+qt2.distance(qt3));
      
 
      
      
      

      YoFramePose yoWorldFrame = new YoFramePose("yoWorldFrame", worldFrame, registry);
      FramePose3D pose = new FramePose3D(worldFrame);
      pose.setPosition(0.0, 0.0, 0.0);
      YoGraphicCoordinateSystem worldFrameGraphic = new YoGraphicCoordinateSystem("world", yoWorldFrame, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("worldGraphic", worldFrameGraphic);
      worldFrameGraphic.setPose(pose);
      
      YoGraphicCoordinateSystem randomSpatialGraphic = new YoGraphicCoordinateSystem("randomSpatial", yoWorldFrame, 0.3);
      yoGraphicsListRegistry.registerYoGraphic("randomSpatialGraphic", randomSpatialGraphic);
      
      //YoGraphicCylinder asymptoteGraphic = new YoGraphicCylinder("asymptote", yoWorldFrame, 0.3);
      
      
      
      
      
      
      

      int numberOfWaypoints = 200;
      double t = 0.0;

      double timeBetweenWaypoints = trajectoryDuration / numberOfWaypoints;
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         t += timeBetweenWaypoints;
      }

      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      for (t = 0.0; t <= trajectoryDuration; t += dt)
      {
         robot.getYoTime().set(t);

         scs.tickAndUpdate();
      }

      System.out.println("END");
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new RigidBodyExplorationVisualizer();
   }

   private static Graphics3DObject createManifoldMessageVisualization(ReachingManifoldMessage reachingMessage, double radius, AppearanceDefinition appearance)
   {
      int configurationValueResolution = 20;
      int numberOfPoints = (int) Math.pow(configurationValueResolution, reachingMessage.manifoldConfigurationSpaces.length);
      int radialResolution = 16;

      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfPoints, radialResolution, radius);

      Point3D[] points = new Point3D[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Pose3D originPose = new Pose3D(reachingMessage.manifoldOriginPosition, reachingMessage.manifoldOriginOrientation);
         double[] configurationValues = new double[reachingMessage.manifoldConfigurationSpaces.length];
         int[] configurationIndex = new int[reachingMessage.manifoldConfigurationSpaces.length];

         int tempIndex = i;
         for (int j = reachingMessage.manifoldConfigurationSpaces.length; j > 0; j--)
         {
            configurationIndex[j - 1] = (int) (tempIndex / Math.pow(configurationValueResolution, j - 1));
            tempIndex = (int) (tempIndex % Math.pow(configurationValueResolution, j - 1));
         }

         for (int j = 0; j < reachingMessage.manifoldConfigurationSpaces.length; j++)
         {
            configurationValues[j] = (reachingMessage.manifoldUpperLimits[j] - reachingMessage.manifoldLowerLimits[j]) / (configurationValueResolution - 1)
                  * configurationIndex[j] + reachingMessage.manifoldLowerLimits[j];
            switch (reachingMessage.manifoldConfigurationSpaces[j])
            {
            case X:
               originPose.appendTranslation(configurationValues[j], 0.0, 0.0);
               break;
            case Y:
               originPose.appendTranslation(0.0, configurationValues[j], 0.0);
               break;
            case Z:
               originPose.appendTranslation(0.0, 0.0, configurationValues[j]);
               break;
            case ROLL:
               originPose.appendRollRotation(configurationValues[j]);
               break;
            case PITCH:
               originPose.appendPitchRotation(configurationValues[j]);
               break;
            case YAW:
               originPose.appendYawRotation(configurationValues[j]);
               break;
            default:
               break;
            }
         }

         points[i] = new Point3D(originPose.getPosition());
      }

      segmentedLine3DMeshGenerator.compute(points);

      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, appearance);
      }

      return graphics;
   }

   private static ArrayList<Graphics3DObject> createExploringRigidBodyVisualization(ExploringRigidBody exploringRigidBody)
   {
      double height = 0.015;
      double radius = 0.003;
      
      ArrayList<Graphics3DObject> graphics = new ArrayList<>();

      RotationMatrix rotationMatrixX = new RotationMatrix(exploringRigidBody.getSpatialData().getOrientation());
      RotationMatrix rotationMatrixY = new RotationMatrix(exploringRigidBody.getSpatialData().getOrientation());
      RotationMatrix rotationMatrixZ = new RotationMatrix(exploringRigidBody.getSpatialData().getOrientation());

      Graphics3DObject point = new Graphics3DObject();
      point.translate(exploringRigidBody.getSpatialData().getPosition());
      point.addSphere(radius, YoAppearance.Black());

      Graphics3DObject zAxis = new Graphics3DObject();
      zAxis.translate(exploringRigidBody.getSpatialData().getPosition());
      zAxis.rotate(rotationMatrixZ);
      zAxis.addCylinder(height, radius, YoAppearance.Blue());

      Graphics3DObject yAxis = new Graphics3DObject();
      yAxis.translate(exploringRigidBody.getSpatialData().getPosition());
      rotationMatrixY.appendRollRotation(-0.5 * Math.PI);
      yAxis.rotate(rotationMatrixY);
      yAxis.addCylinder(height, radius, YoAppearance.Green());

      Graphics3DObject xAxis = new Graphics3DObject();
      xAxis.translate(exploringRigidBody.getSpatialData().getPosition());
      rotationMatrixX.appendPitchRotation(0.5 * Math.PI);
      xAxis.rotate(rotationMatrixX);
      xAxis.addCylinder(height, radius, YoAppearance.Red());

      graphics.add(point);
      graphics.add(zAxis);
      graphics.add(yAxis);
      graphics.add(xAxis);

      return graphics;
   }

   private static Graphics3DObject createConnectionVisualization(ExploringRigidBody exploringRigidBodyStart, ExploringRigidBody exploringRigidBodyGoal)
   {
      Graphics3DObject connection = new Graphics3DObject();

      double distance = exploringRigidBodyStart.getPositionDistance(exploringRigidBodyGoal);

      Vector3D zAxis = new Vector3D();
      zAxis.setX(exploringRigidBodyGoal.getSpatialData().getPosition().getX() - exploringRigidBodyStart.getSpatialData().getPosition().getX());
      zAxis.setY(exploringRigidBodyGoal.getSpatialData().getPosition().getY() - exploringRigidBodyStart.getSpatialData().getPosition().getY());
      zAxis.setZ(exploringRigidBodyGoal.getSpatialData().getPosition().getZ() - exploringRigidBodyStart.getSpatialData().getPosition().getZ());

      Vector3D yAxis = new Vector3D();
      yAxis.cross(new Vector3D(0, 0, 1), zAxis);

      Vector3D xAxis = new Vector3D();
      xAxis.cross(yAxis, zAxis);
      xAxis.normalize();
      yAxis.normalize();
      zAxis.normalize();

      RotationMatrix rotation = new RotationMatrix();
      rotation.setColumns(xAxis, yAxis, zAxis);

      connection.translate(exploringRigidBodyStart.getSpatialData().getPosition());
      connection.rotate(rotation);
      connection.addCylinder(distance, 0.003, YoAppearance.AliceBlue());

      return connection;
   }
}

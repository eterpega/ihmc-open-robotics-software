package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

public class WholeBodyTrajectoryToolboxMessageTools
{
   public static interface FunctionTrajectory
   {
      public Pose3D compute(double time);
   }

   public static FunctionTrajectory createFunctionTrajectory(WaypointBasedTrajectoryMessage message)
   {
      return new FunctionTrajectory()
      {
         @Override
         public Pose3D compute(double time)
         {
            Pose3D current = new Pose3D();

            Pose3D previous = null;
            Pose3D next = null;
            double t0 = Double.NaN;
            double tf = Double.NaN;

            for (int i = 1; i < message.getNumberOfWaypoints(); i++)
            {
               t0 = message.getWaypointTime(i - 1);
               tf = message.getWaypointTime(i);
               previous = message.getWaypoint(i - 1);
               next = message.getWaypoint(i);
               if (time < message.getWaypointTime(i))
                  break;
            }

            double alpha = (time - t0) / (tf - t0);
            alpha = MathTools.clamp(alpha, 0.0, 1.0);
            current.interpolate(previous, next, alpha);

            return current;
         }
      };
   }

   public static WaypointBasedTrajectoryMessage createTrajectoryMessage(RigidBody endEffector, double t0, double tf, double timeResolution,
                                                                        FunctionTrajectory trajectoryToDiscretize, SelectionMatrix6D selectionMatrix)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      // Adjust the timeResolution using the numberOfWaypoints:
      timeResolution = (tf - t0) / (numberOfWaypoints - 1);

      double[] waypointTimes = new double[numberOfWaypoints];
      Pose3D[] waypoints = new Pose3D[numberOfWaypoints];

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double waypointTime = i * timeResolution + t0;

         waypointTimes[i] = waypointTime;
         waypoints[i] = trajectoryToDiscretize.compute(waypointTime);
      }

      return new WaypointBasedTrajectoryMessage(endEffector, waypointTimes, waypoints, selectionMatrix);
   }

   public static double[] createDefaultExplorationAmplitudeArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] lowerLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         lowerLimit[i] = configurationSpaceNames[i].getDefaultExplorationAmplitude();
      return lowerLimit;
   }

   public static double[] createDefaultExplorationUpperLimitArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] upperLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         upperLimit[i] = configurationSpaceNames[i].getDefaultExplorationUpperLimit();
      return upperLimit;
   }

   public static double[] createDefaultExplorationLowerLimitArray(ConfigurationSpaceName... configurationSpaceNames)
   {
      double[] lowerLimit = new double[configurationSpaceNames.length];
      for (int i = 0; i < configurationSpaceNames.length; i++)
         lowerLimit[i] = configurationSpaceNames[i].getDefaultExplorationLowerLimit();
      return lowerLimit;
   }

   public static double computePoseDistance(Pose3D poseOne, Pose3D poseTwo, double positionWeight, double orientationWeight)
   {
      double distance = 0.0;

      double positionDistance = poseOne.getPositionDistance(poseTwo);
      double orientationDistance = poseOne.getOrientationDistance(poseTwo);
      orientationDistance = AngleTools.trimAngleMinusPiToPi(orientationDistance);
      orientationDistance = Math.abs(orientationDistance);
      distance = positionWeight * positionDistance + orientationWeight * orientationDistance;

      return distance;
   }
   
   /**
    * Manifold message for Sphere.
    * 
    * @param rigidBody : which rigid body to reach on this sphere.
    * @param originPosition : origin position of this sphere.
    * @param radius : radius of this sphere.
    */
   public static ReachingManifoldMessage createSphereManifoldMessages(RigidBody rigidBody, Point3D originPosition, double radius)
   {
      ReachingManifoldMessage manifoldMessage = new ReachingManifoldMessage(rigidBody);

      manifoldMessage.setOrigin(originPosition, new Quaternion());

      ConfigurationSpaceName[] manifoldSpaces = {YAW, PITCH, ConfigurationSpaceName.X};
      double[] lowerLimits = new double[] {-Math.PI*0.5, -Math.PI * 0.5, -radius};
      double[] upperLimits = new double[] {Math.PI*0.5, Math.PI * 0.5, radius};
      manifoldMessage.setManifold(manifoldSpaces, lowerLimits, upperLimits);

      return manifoldMessage;
   }

   /**
    * Manifold message for Cylinder.
    * Cylinder will be created from the bottom.
    * 
    * @param rigidBody : which rigid body to reach on this cylinder.
    * @param originPosition : origin position of this cylinder.
    * @param originOrientation : origin orientation of this cylinder.
    * @param radius : radius of this cylinder.
    * @param height : height of this cylinder.
    * @return
    */
   public static ReachingManifoldMessage createCylinderManifoldMessages(RigidBody rigidBody, Point3D originPosition, Quaternion originOrientation,
                                                                        double radius, double height)
   {
      ReachingManifoldMessage manifoldMessage = new ReachingManifoldMessage(rigidBody);

      manifoldMessage.setOrigin(originPosition, originOrientation);

      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.YAW, ConfigurationSpaceName.X, ConfigurationSpaceName.Z};
      double[] lowerLimits = new double[] {-Math.PI*0.5, -radius, 0.0};
      double[] upperLimits = new double[] {Math.PI*0.5, radius, height};
      manifoldMessage.setManifold(manifoldSpaces, lowerLimits, upperLimits);

      return manifoldMessage;
   }

   /**
    * Manifold message for Box.
    * Box will be created from the center.
    * 
    * @param rigidBody : which rigid body to reach on this box.
    * @param originPosition : origin position of this box.
    * @param originOrientation : origin orientation of this box.
    * @param lengthX : length in X of this box.
    * @param lengthY : length in Y of this box.
    * @param lengthZ : length in Z of this box.
    * @return
    */
   public static ReachingManifoldMessage createBoxManifoldMessages(RigidBody rigidBody, Point3D originPosition, Quaternion originOrientation, double lengthX,
                                                                   double lengthY, double lengthZ)
   {
      ReachingManifoldMessage manifoldMessage = new ReachingManifoldMessage(rigidBody);

      manifoldMessage.setOrigin(originPosition, originOrientation);

      ConfigurationSpaceName[] manifoldSpaces = {ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z};
      double[] lowerLimits = new double[] {-lengthX * 0.5, -lengthY * 0.5, -lengthZ * 0.5};
      double[] upperLimits = new double[] {lengthX * 0.5, lengthY * 0.5, lengthZ * 0.5};
      manifoldMessage.setManifold(manifoldSpaces, lowerLimits, upperLimits);

      return manifoldMessage;
   }
}

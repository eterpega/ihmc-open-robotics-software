package us.ihmc.robotics.immutableRobotDescription;

import java.util.Collection;

public interface JointDescription extends NamedObject, LocatedObject
{
   Collection<JointDescription> getChildrenJoints();

   LinkDescription getLink();

   Collection<GroundContactPointDescription> getGroundContactPoints();

   Collection<ExternalForcePointDescription> getExternalForcePoints();

   Collection<KinematicPointDescription> getKinematicPoints();

   Collection<JointWrenchSensorDescription> getWrenchSensors();

   Collection<CameraSensorDescription> getCameraSensors();

   Collection<IMUSensorDescription> getIMUSensors();

   Collection<LidarSensorDescription> getLidarSensors();

   Collection<ForceSensorDescription> getForceSensors();

   boolean getDynamic();
}

package us.ihmc.robotics.immutableRobotDescription;

// TODO: GroundContactPointDescription extends this but also gets instantiated in SDF loader...
public interface ExternalForcePointDescription extends KinematicPointDescription
{
   abstract class Builder implements JointDescription.Builder {}
}

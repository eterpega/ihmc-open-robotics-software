package us.ihmc.robotics.immutableRobotDescription;

/**
 * Interface for immutable objects that also have their mutable counterparts.
 */
public interface ModifiableObject
{
   public ModifiableObject toModifiable();
}

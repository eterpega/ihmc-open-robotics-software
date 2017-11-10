package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

public class CollisionAvoidanceModuleSettings
{
   private double epsilon;
   private double collisionAvoidanceVelocity;
   
   public void setCollisionDetectionEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }
   
   public double getCollisionDetectionEpsilon()
   {
      return epsilon;
   }
   
   public void setCollisionAvoidanceVelocity(double velocityToSet)
   {
      this.collisionAvoidanceVelocity = velocityToSet;
   }
   
   public double getCollisionAvoidanceVelocity()
   {
      return collisionAvoidanceVelocity;
   }
}

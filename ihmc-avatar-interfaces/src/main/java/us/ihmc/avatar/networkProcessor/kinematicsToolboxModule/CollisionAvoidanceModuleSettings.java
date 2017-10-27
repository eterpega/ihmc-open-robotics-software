package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

public class CollisionAvoidanceModuleSettings
{
   private double epsilon;
   
   public void setCollisionDetectionEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }
   
   public double getCollisionDetectionEpsilon()
   {
      return epsilon;
   }
}

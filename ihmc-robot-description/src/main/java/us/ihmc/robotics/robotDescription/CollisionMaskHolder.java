package us.ihmc.robotics.robotDescription;

public interface CollisionMaskHolder
{
   public abstract int getCollisionGroup();
   public abstract void setCollisionGroup(int collisionGroup);

   public abstract int getCollisionMask();
   public abstract void setCollisionMask(int collisionMask);
   
   public default boolean isMatched(CollisionMaskHolder other)
   {
      if ((getCollisionGroup() & other.getCollisionMask()) == 0x00)
         return false;
      if ((getCollisionMask() & other.getCollisionGroup()) == 0x00)
         return false;

      return true;
   }
}

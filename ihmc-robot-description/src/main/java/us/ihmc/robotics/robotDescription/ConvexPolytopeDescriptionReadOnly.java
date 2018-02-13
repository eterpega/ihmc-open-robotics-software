package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.geometry.polytope.ConvexPolytope;

public class ConvexPolytopeDescriptionReadOnly implements ConvexShapeDescriptionReadOnly
{
   //TODO: Trying to create a read only version by copying a full version. Should be a more efficient way to do this...
   private final ConvexPolytope convexPolytope;

   public ConvexPolytopeDescriptionReadOnly(ConvexPolytope polytope, RigidBodyTransform rigidBodyTransform)
   {
      this.convexPolytope = new ConvexPolytope(polytope);
      this.convexPolytope.applyTransform(rigidBodyTransform);
   }

   public ConvexPolytope getConvexPolytope()
   {
      return new ConvexPolytope(convexPolytope);
   }

}

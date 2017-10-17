package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;

public abstract class ConvexPolytopeBasics implements GeometryObject<ConvexPolytopeBasics>
{
   private ConvexPolytopeFaceProvider faceProvider;
   
   public ConvexPolytopeBasics(ConvexPolytopeFaceProvider faceProvider)
   {
      // TODO Auto-generated constructor stub
   }
   
   @Override
   public void applyTransform(Transform transform)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean epsilonEquals(ConvexPolytopeBasics other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void set(ConvexPolytopeBasics other)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean containsNaN()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void setToNaN()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setToZero()
   {
      // TODO Auto-generated method stub
      
   }
   
}

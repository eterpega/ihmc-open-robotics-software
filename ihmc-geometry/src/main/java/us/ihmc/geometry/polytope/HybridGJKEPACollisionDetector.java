package us.ihmc.geometry.polytope;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class HybridGJKEPACollisionDetector
{
   private static final Point3D origin = new Point3D();
   private double epsilon = Epsilons.ONE_TEN_THOUSANDTH;

   private final ExtendedSimplexPolytope simplex = new ExtendedSimplexPolytope();
   private Vector3D supportVectorDirectionNegative = new Vector3D();
   private Vector3D supportVectorDirection = new Vector3D()
   {
      @Override
      public final void setX(double x)
      {
         super.setX(x);
         supportVectorDirectionNegative.setX(-x);
      };
      @Override
      public final void setY(double y)
      {
         super.setY(y);
         supportVectorDirectionNegative.setY(-y);
      };
      @Override
      public final void setZ(double x)
      {
         super.setZ(x);
         supportVectorDirectionNegative.setZ(-x);
      };
   };
   private Vector3D previousSupportVectorDirection = new Vector3D();

   private final int iterations = 10;

   public void setSupportVectorDirection(Vector3DReadOnly vectorToSet)
   {
      supportVectorDirection.set(vectorToSet);
   }
   
   public void getSupportVectorDirection(Vector3D vectorToPack)
   {
      vectorToPack.set(supportVectorDirection);
   }

   public void getSupportVectorDirectionNegative(Vector3D vectorToPack)
   {
      vectorToPack.set(supportVectorDirectionNegative);
   }
   
   public HybridGJKEPACollisionDetector()
   {
      this(Epsilons.ONE_BILLIONTH);
   }

   public HybridGJKEPACollisionDetector(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public boolean checkCollisionBetweenTwoPolytopes(ConvexPolytope polytopeA, ConvexPolytope polytopeB, Vector3D initialDirectionForSearch)
   {
      simplex.clear();
      setSupportVectorDirection(initialDirectionForSearch);
      simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
      simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
      previousSupportVectorDirection.set(supportVectorDirection);
      double prevDistanceToGo = Double.NaN;
      for (int i = 0; i < iterations;)
      {
         simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
         double distanceToGo = simplex.getShortestDistanceTo(origin);
         if(distanceToGo <= epsilon)
         {
            //PrintTools.debug("Distance to go: " + distanceToGo + "\n" + simplex.toString());
            return true;
         }
         else
            simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
         if(prevDistanceToGo - distanceToGo < epsilon)
            i++;
         if(previousSupportVectorDirection.epsilonEquals(supportVectorDirection, epsilon))
            return false;
         else
            previousSupportVectorDirection.set(supportVectorDirection);
      }
      return false;
   }
   
   public void runEPAExpansion(ConvexPolytope polytopeA, ConvexPolytope polytopeB, Vector3D collisionVectorToPack)
   {
      //PrintTools.debug(supportVectorDirection.toString());
      runEPAExpansion(polytopeA, polytopeB, simplex, supportVectorDirection, collisionVectorToPack);
   }

   public void runEPAExpansion(ConvexPolytope polytopeA, ConvexPolytope polytopeB, ExtendedSimplexPolytope simplex, Vector3D initialSupportVectorDirection, Vector3D collisionVectorToPack)
   {
      supportVectorDirection.set(initialSupportVectorDirection);
      previousSupportVectorDirection.set(initialSupportVectorDirection);
      while(true)
      {
         simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
         simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
         if(supportVectorDirection.epsilonEquals(previousSupportVectorDirection, epsilon))
            break;
         else
            previousSupportVectorDirection.set(supportVectorDirection);
      }
      collisionVectorToPack.set(supportVectorDirection);
      collisionVectorToPack.normalize();
      collisionVectorToPack.scale(simplex.getSmallestSimplexMemberReference(origin).getShortestDistanceTo(origin));
   }
   
   public ConvexPolytope getSimplex()
   {
      return simplex.getPolytope();
   }
}

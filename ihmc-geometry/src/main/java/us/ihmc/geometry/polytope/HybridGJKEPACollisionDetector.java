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

   private final SimplexPolytope simplex = new SimplexPolytope();
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

   public void checkCollisionBetweenTwoPolytopes(ConvexPolytope polytopeA, ConvexPolytope polytopeB, ExtendedSimplexPolytope simplex,
                                                 Vector3D initialDirectionForSearch)
   {
      simplex.clear();
      initialDirectionForSearch.negate();
      simplex.addVertex(polytopeA.getSupportingVertexHack(initialDirectionForSearch), polytopeB.getSupportingVertexHack(initialDirectionForSearch));
      simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
      double prevDistanceToGo = Double.NaN;
      for (int i = 0; i < iterations;)
      {
         simplex.addVertex(polytopeA.getSupportingVertexHack(supportVectorDirection), polytopeB.getSupportingVertexHack(supportVectorDirectionNegative));
         double distanceToGo = simplex.getShortestDistanceTo(origin) ;
         if(distanceToGo <= epsilon)
         {
            PrintTools.debug("Got collision");
            break;
         }
         else
            simplex.getSupportVectorDirectionTo(origin, supportVectorDirection);
         if(prevDistanceToGo - distanceToGo < epsilon)
            i++;
      }
   }
}

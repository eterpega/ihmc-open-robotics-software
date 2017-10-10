package us.ihmc.geometry.polytope;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class HybridGJKEPACollisionDetector
{
   private final SimplexPolytope simplex = new SimplexPolytope();
   private double epsilon = Epsilons.ONE_TEN_THOUSANDTH;

   public HybridGJKEPACollisionDetector()
   {
      this(Epsilons.ONE_BILLIONTH);
   }

   public HybridGJKEPACollisionDetector(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public static void checkCollisionBetweenTwoPolytopes(ConvexPolytope polytopeA, ConvexPolytope polytopeB, ExtendedSimplexPolytope simplex,
                                                        Vector3D initialDirectionForSearch)
   {
      simplex.clear();
      Point3D point1 = polytopeA.getSupportingVertex(initialDirectionForSearch);
      initialDirectionForSearch.negate();
      Point3D point2 = polytopeB.getSupportingVertex(initialDirectionForSearch);
      //SimplexVertex newVertex = new SimplexVertex(point1, point2);
   }
}

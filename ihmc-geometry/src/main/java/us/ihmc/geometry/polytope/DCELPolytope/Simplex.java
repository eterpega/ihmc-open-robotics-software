package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.SimplexBasics;

public interface Simplex extends SimplexBasics
{
   default Simplex getSmallestSimplexMemberReference(Point3D point)
   {
      return (Simplex) getSmallestSimplexMemberReference(point);
   }
}

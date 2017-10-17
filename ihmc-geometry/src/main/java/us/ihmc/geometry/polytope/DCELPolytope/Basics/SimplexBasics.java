package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface SimplexBasics<Q extends SimplexBasics<Q>>
{
   double getShortestDistanceTo(Point3DReadOnly point);
   void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack);
   void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack);
   SimplexBasics<Q> getSmallestSimplexMemberReference(Point3DReadOnly point);
}

package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface SupportingVertexHolderBasics
{
   Point3DReadOnly getSupportingVertex(Vector3D supportDirection);
}

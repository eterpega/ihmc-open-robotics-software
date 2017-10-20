package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.SupportingVertexHolder;

public interface ConvexPolytopeReadOnly extends EpsilonComparable<ConvexPolytopeReadOnly>, SupportingVertexHolder 
{
   List<? extends ConvexPolytopeFaceReadOnly> getFaces();
   PolytopeVertexBasics getSupportingVertexHack(Vector3DReadOnly supportingVertexDirection);
}

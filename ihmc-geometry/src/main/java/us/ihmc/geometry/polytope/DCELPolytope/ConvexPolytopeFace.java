package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeBuilder;

/**
 * This class defines a polytope face. A face is defined by the set of edges that bound it.
 * 
 * @author Apoorv S
 *
 */
public class ConvexPolytopeFace extends ConvexPolytopeFaceBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex> implements Simplex, SupportingVertexHolder
{
   public ConvexPolytopeFace()
   {
      super(new PolytopeHalfEdgeBuilder());
   }

   @Override
   protected ConvexPolytopeFace getThis()
   {
      return this;
   }
}

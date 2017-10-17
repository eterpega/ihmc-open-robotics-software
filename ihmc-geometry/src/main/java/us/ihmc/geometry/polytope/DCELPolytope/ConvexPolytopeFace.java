package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

/**
 * This class defines a polytope face. A face is defined by the set of edges that bound it.
 * 
 * @author Apoorv S
 *
 */
public class ConvexPolytopeFace extends ConvexPolytopeFaceBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex>
{
   private final PolytopeHalfEdgeBuilder halfEdgeBuilder = new PolytopeHalfEdgeBuilder();
   
   public ConvexPolytopeFace()
   {
      super();
   }
   
   public ConvexPolytopeFace(PolytopeHalfEdge[] edgeList)
   {
      this();
      copyEdgeList(edgeList);
   }

   @Override
   protected ConvexPolytopeFace getThis()
   {
      return this;
   }

   @Override
   protected PolytopeHalfEdgeProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex> getHalfEdgeProvider()
   {
      return halfEdgeBuilder;
   }
}

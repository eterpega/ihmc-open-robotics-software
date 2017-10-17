package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexProvider;

/**
 * A convex polytope is a collection of faces that describe it 
 * 
 * This class is a data structure for storing a polytope in the DCEL notation (ref: https://en.wikipedia.org/wiki/Doubly_connected_edge_list).
 * Based on the original implementation by Jerry Pratt
 * @author Apoorv S
 */

public class ExtendedConvexPolytope extends ConvexPolytopeBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, ExtendedConvexPolytope, Simplex> 
{
   private final ConvexPolytopeFaceBuilder faceBuilder = new ConvexPolytopeFaceBuilder(); 
   private final PolytopeVertexBuilder vertexBuilder =  new PolytopeVertexBuilder();
   public ExtendedConvexPolytope()
   {
      super();
   }
   
   public ExtendedConvexPolytope(ExtendedConvexPolytope polytope)
   {
      super(polytope);
   }

   @Override
   protected PolytopeVertexProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex> getVertexProvider()
   {
      return vertexBuilder;
   }

   @Override
   protected ConvexPolytopeFaceProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace, Simplex> getConvexFaceProvider()
   {
      return faceBuilder;
   }
}

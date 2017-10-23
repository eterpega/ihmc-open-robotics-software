package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ExtendedSimplexPolytope implements Simplex
{
   private double epsilon = Epsilons.ONE_TEN_THOUSANDTH;
   private ExtendedConvexPolytope polytope = new ExtendedConvexPolytope();
   RecyclingArrayList<SimplexVertex> vertices = new RecyclingArrayList<>(SimplexVertex.class);
   
   
   public ExtendedSimplexPolytope()
   {
      super();
   }
   
   public void setEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
   }
   
   public void addVertex(PolytopeVertexReadOnly vertexOnPolytopeA, PolytopeVertexReadOnly vertexOnPolytopeB)
   {
      addVertex(vertexOnPolytopeA, vertexOnPolytopeB, epsilon);
   }
   
   public void addVertex(PolytopeVertexReadOnly vertexOnPolytopeA, PolytopeVertexReadOnly vertexOnPolytopeB, double epsilon)
   {
      SimplexVertex newVertex = vertices.add();
      newVertex.set(vertexOnPolytopeA, vertexOnPolytopeB);
      polytope.addVertex(newVertex, epsilon);
   }

   public void clear()
   {
      vertices.clear();
      polytope.clear();
   }

   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      return polytope.getShortestDistanceTo(point);
   }

   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      polytope.getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   public Simplex getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return (Simplex) polytope.getSmallestSimplexMemberReference(point);
   }
   
   public String toString()
   {
      return polytope.toString();
   }

   public ExtendedConvexPolytope getPolytope()
   {
      return polytope;
   }
   
   public void getCollidingPointsOnSimplex(Point3DReadOnly point, Point3D pointOnA, Point3D pointOnB)
   {
      Simplex member = getSmallestSimplexMemberReference(point);
      // Assuming linearity between the simplex and polytope points 
      if(member instanceof ConvexPolytopeFace)
      {
         // TODO fix this nasty type casting 
         SimplexVertex simplexVertex1 = (SimplexVertex) ((ConvexPolytopeFace)member).getEdge(0).getOriginVertex();
         PolytopeVertexReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         SimplexVertex simplexVertex2 = (SimplexVertex) ((ConvexPolytopeFace) member).getEdge(0).getDestinationVertex();
         PolytopeVertexReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         SimplexVertex simplexVertex3 = (SimplexVertex) ((ConvexPolytopeFace) member).getEdge(1).getDestinationVertex();
         PolytopeVertexReadOnly polytopeAVertex3 = simplexVertex3.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex3 = simplexVertex3.getVertexOnPolytopeB();
         
         // Computing the coordinate vector for the face basis (using the first two edges as the basis)
         
      }
      else if (member instanceof PolytopeHalfEdge)
      {
         // TODO fix this nasty type casting 
         SimplexVertex simplexVertex1 = (SimplexVertex) ((PolytopeHalfEdge) member).getOriginVertex();
         PolytopeVertexReadOnly polytopeAVertex1 = simplexVertex1.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex1 = simplexVertex1.getVertexOnPolytopeB();
         SimplexVertex simplexVertex2 = (SimplexVertex) ((PolytopeHalfEdge) member).getDestinationVertex();
         PolytopeVertexReadOnly polytopeAVertex2 = simplexVertex2.getVertexOnPolytopeA();
         PolytopeVertexReadOnly polytopeBVertex2 = simplexVertex2.getVertexOnPolytopeB();
         double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, simplexVertex1, simplexVertex2);
         pointOnA.interpolate(polytopeAVertex1, polytopeAVertex2, percentage);
         pointOnB.interpolate(polytopeBVertex1, polytopeBVertex2, percentage);
      }
      else if (member instanceof SimplexVertex)
      {
         pointOnA.set(((SimplexVertex) member).getVertexOnPolytopeA());
         pointOnB.set(((SimplexVertex) member).getVertexOnPolytopeB());
      }
      else
      {
         throw new RuntimeException("Unhandled simplex member " + member.getClass());
      }
   }
}

package us.ihmc.geometry.polytope.DCELPolytope;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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
   
   public void addVertex(ExtendedPolytopeVertex vertexOnPolytopeA, ExtendedPolytopeVertex vertexOnPolytopeB)
   {
      addVertex(vertexOnPolytopeA, vertexOnPolytopeB, epsilon);
   }
   
   public void addVertex(ExtendedPolytopeVertex vertexOnPolytopeA, ExtendedPolytopeVertex vertexOnPolytopeB, double epsilon)
   {
      SimplexVertex newVertex = vertices.add();
      newVertex.set(vertexOnPolytopeA, vertexOnPolytopeB);
      polytope.addVertex(newVertex, epsilon);
   }

   public void clear()
   {
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

   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      polytope.getSupportVectorJacobianTo(point, jacobianToPack);
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
}

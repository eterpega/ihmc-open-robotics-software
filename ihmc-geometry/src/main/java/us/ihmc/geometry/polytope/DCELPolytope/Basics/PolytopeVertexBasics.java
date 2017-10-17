package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Simplex;

public abstract class PolytopeVertexBasics<T extends PolytopeVertexBasics<T, S, U, Q>, S extends PolytopeHalfEdgeBasics<T, S, U, Q>, U extends ConvexPolytopeFaceBasics<T, S, U, Q>, Q extends SimplexBasics<Q>>
      implements GeometryObject<T>, Point3DBasics, SimplexBasics<Q>
{
   protected abstract Point3DBasics getPosition();
   private final ArrayList<S> associatedEdges = new ArrayList<>();
   private DenseMatrix64F jacobian;

   public PolytopeVertexBasics()
   {
   }

   @Override
   public void set(T vertex)
   {
      getPosition().set(vertex.getPosition());
      clearAssociatedEdgeList();
      copyEdges(vertex.getAssociatedEdges());
   }

   public List<S> getAssociatedEdges()
   {
      return associatedEdges;
   }

   public S getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   public void removeAssociatedEdge(S edgeToAdd)
   {
      associatedEdges.remove(edgeToAdd);
   }

   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   public void copyEdges(List<S> edgeList)
   {
      for (int i = 0; i < edgeList.size(); i++)
      {
         addAssociatedEdge(edgeList.get(i));
      }
   }

   public void addAssociatedEdge(S edge)
   {
      if (!isAssociatedWithEdge(edge))
         associatedEdges.add(edge);
   }

   public boolean isAssociatedWithEdge(S edgeToCheck)
   {
      return associatedEdges.contains(edgeToCheck);
   }

   public boolean isAssociatedWithEdge(S edgeToCheck, double epsilon)
   {
      boolean result = associatedEdges.size() > 0;
      for (int i = 0; result && i < associatedEdges.size(); i++)
      {
         result &= associatedEdges.get(i).epsilonEquals(edgeToCheck, epsilon);
      }
      return result;
   }

   public int getNumberOfAssociatedEdges()
   {
      return associatedEdges.size();
   }

   public double dot(Vector3DReadOnly vector)
   {
      return getX() * vector.getX() + getY() * vector.getY() + getZ() * vector.getZ();
   }

   public String toString()
   {
      return "( " + getX() + ", " + getY() + ", " + getZ() + ")";
   }

   @Override
   public double getX()
   {
      return getPosition().getX();
   }

   @Override
   public double getY()
   {
      return getPosition().getY();
   }

   @Override
   public double getZ()
   {
      return getPosition().getZ();
   }

   public double getElement(int index)
   {
      return getPosition().getElement(index);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(getPosition());
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPosition());
   }

   public boolean isAnyFaceMarked()
   {
      boolean isMarked = false;
      for (int i = 0; !isMarked && i < associatedEdges.size(); i++)
      {
         isMarked |= associatedEdges.get(i).getFace().isMarked();
      }
      //PrintTools.debug(toString() + " " +isMarked);
      return isMarked;
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      return getPosition().epsilonEquals(other.getPosition(), epsilon);
   }

   @Override
   public boolean containsNaN()
   {
      return getPosition().containsNaN();
   }

   @Override
   public void setToNaN()
   {
      getPosition().setToNaN();
   }

   @Override
   public void setToZero()
   {
      getPosition().setToZero();
   }

   @Override
   public void setX(double x)
   {
      getPosition().setX(x);
   }

   @Override
   public void setY(double y)
   {
      getPosition().setY(y);
   }

   @Override
   public void setZ(double z)
   {
      getPosition().setZ(z);
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      double dx = getX() - point.getX();
      double dy = getX() - point.getX();
      double dz = getX() - point.getX();
      return Math.sqrt((dx * dx) + (dy * dy) + (dz * dz));
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      supportVectorToPack.sub(point, this);
   }

   @Override
   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.set(jacobian);
      CommonOps.scale(-1.0, jacobianToPack);
   }

   public void setJacobian(DenseMatrix64F jacobian)
   {
      this.jacobian.set(jacobian);
   }

   public DenseMatrix64F getJacobian()
   {
      return jacobian;
   }

   @Override
   public SimplexBasics<Q> getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      return this;
   }
}

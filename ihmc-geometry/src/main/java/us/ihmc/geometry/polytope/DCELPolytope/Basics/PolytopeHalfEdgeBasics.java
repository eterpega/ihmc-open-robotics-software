package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;

public abstract class PolytopeHalfEdgeBasics<T extends PolytopeVertexBasics<T, S, U, Q>, S extends PolytopeHalfEdgeBasics<T, S, U, Q>, U extends ConvexPolytopeFaceBasics<T, S, U, Q>, Q extends SimplexBasics<Q>>
      implements GeometryObject<S>, SimplexBasics<Q>, SupportingVertexHolderBasics, Vector3DReadOnly
{
   private S twinEdge;
   private S nextHalfEdge;
   private S previousHalfEdge;
   private U face;
   private T originVertex;
   private T destinationVertex;
   /**
    * Not recomputed on change of values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();
   private Point3D tempPoint = new Point3D();
   private DenseMatrix64F jacobian = new DenseMatrix64F();

   public PolytopeHalfEdgeBasics()
   {

   }

   /**
    * Primary constructor for half edge
    * @param originVertex
    * @param destinationVertex
    */
   public PolytopeHalfEdgeBasics(T originVertex, T destinationVertex)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
   }

   public PolytopeHalfEdgeBasics(S edge)
   {
      set(edge);
   }

   /**
    * Function for creating the twin edge. Generates garbage. Use {@code getTwinHalfEdge()} to get the twin half edge
    * @return
    */
   public S createTwinHalfEdge()
   {
      return createTwinHalfEdge(null);
   }

   public abstract S createTwinHalfEdge(ConvexPolytopeFace twinEdgeFace);

   /**
    * Function for creating the twin edge and storing its value. Generates garbage. Use {@code getTwinHalfEdge()} to get the twin half edge
    * @return
    */
   public S setAndCreateTwinHalfEdge(ConvexPolytopeFace face)
   {
      this.twinEdge = createTwinHalfEdge(face);
      return this.twinEdge;
   }

   public S setAndCreateTwinHalfEdge()
   {
      return setAndCreateTwinHalfEdge(null);
   }

   public void setToTwinOf(S twinEdge)
   {
      twinEdge.clear();
      twinEdge.setOriginVertex(this.destinationVertex);
      twinEdge.setDestinationVertex(this.originVertex);
      twinEdge.setTwinHalfEdge(getThis());
   }
   // @Override
   // public Q getSmallestSimplexMemberReference(Point3DReadOnly point)
   // {
   //    double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
   //    if (percentage <= 0.0)
   //       return this.originVertex;
   //    else if (percentage >= 1.0)
   //       return this.destinationVertex;
   //    else
   //       return this;

   public PolytopeHalfEdgeBasics(S twinEdge, U face)
   {
      setTwinHalfEdge(twinEdge);
      setOriginVertex(twinEdge.getDestinationVertex());
      setDestinationVertex(twinEdge.getOriginVertex());
      setFace(face);
   }

   public PolytopeHalfEdgeBasics(T originVertex, T destinationVertex, S twinEdge, S nextHalfEdge, S previousHalfEdge, U face)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
      setFace(face);
   }

   public void setOriginVertex(T originVertex)
   {
      if (this.originVertex != null)
         this.originVertex.removeAssociatedEdge(getThis());
      this.originVertex = originVertex;
      if (this.originVertex != null)
         this.originVertex.addAssociatedEdge(getThis());
      updateTwinDestination();
   }

   private void setOriginVertexInternal(T originVertex)
   {
      this.originVertex = originVertex;
   }

   private void updateTwinOrigin()
   {
      if (twinEdge != null)
      {
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().removeAssociatedEdge(twinEdge);
         twinEdge.setOriginVertexInternal(this.destinationVertex);
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().addAssociatedEdge(twinEdge);
      }
   }

   private void updateTwinDestination()
   {
      if (twinEdge != null)
         twinEdge.setDestinationVertexInternal(this.originVertex);
   }

   public T getOriginVertex()
   {
      return originVertex;
   }

   public void setDestinationVertex(T destinationVertex)
   {
      this.destinationVertex = destinationVertex;
      updateTwinOrigin();
   }

   public T getDestinationVertex()
   {
      return destinationVertex;
   }

   public void setTwinHalfEdge(S twinEdge)
   {
      this.twinEdge = twinEdge;
   }

   public S getTwinHalfEdge()
   {
      return twinEdge;
   }

   public void setNextHalfEdge(S nextHalfEdge)
   {
      if (nextHalfEdge == null || (nextHalfEdge.getOriginVertex() == this.getDestinationVertex() && nextHalfEdge.getFace() == this.getFace()))
         setNextHalfEdgeUnsafe(nextHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestinationVertex().toString() + " , next edge origin vertex: "
               + nextHalfEdge.getOriginVertex().toString());
   }

   private void setNextHalfEdgeUnsafe(S nextHalfEdge)
   {
      this.nextHalfEdge = nextHalfEdge;
   }

   public S getNextHalfEdge()
   {
      return nextHalfEdge;
   }

   public void setPreviousHalfEdge(S previousHalfEdge)
   {
      if (previousHalfEdge == null || (previousHalfEdge.getDestinationVertex() == this.getOriginVertex() && previousHalfEdge.getFace() == this.getFace()))
         setPreviousHalfEdgeUnsafe(previousHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOriginVertex().toString() + " , previous edge destination vertex: "
               + previousHalfEdge.getDestinationVertex().toString());
   }

   private void setPreviousHalfEdgeUnsafe(S previousHalfEdge)
   {
      this.previousHalfEdge = previousHalfEdge;
   }

   public S getPreviousHalfEdge()
   {
      return previousHalfEdge;
   }

   public void setFace(U face)
   {
      this.face = face;
   }

   public U getFace()
   {
      return face;
   }

   public Vector3DReadOnly getEdgeVector()
   {
      edgeVector.sub(this.destinationVertex.getPosition(), this.originVertex.getPosition());
      return edgeVector;
   }

   public Vector3DReadOnly getNormalizedEdgeVector()
   {
      getEdgeVector();
      edgeVector.normalize();
      return edgeVector;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      originVertex.applyTransform(transform);
      destinationVertex.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      originVertex.applyInverseTransform(transform);
      destinationVertex.applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(S other, double epsilon)
   {
      return getOriginVertex().epsilonEquals(other.getOriginVertex(), epsilon) && getDestinationVertex().epsilonEquals(other.getDestinationVertex(), epsilon);
   }

   public boolean isTwin(PolytopeHalfEdgeBasics<T, S, U, Q> twinEdge, double epsilon)
   {
      return epsilonEquals(twinEdge.getTwinHalfEdge(), epsilon);
   }

   @Override
   public void set(S other)
   {
      setOriginVertex(other.getOriginVertex());
      setDestinationVertex(other.getDestinationVertex());
      setTwinHalfEdge(other.getTwinHalfEdge());
      setNextHalfEdge(other.getNextHalfEdge());
      setPreviousHalfEdge(other.getPreviousHalfEdge());
      setFace(other.getFace());
   }

   @Override
   public boolean containsNaN()
   {
      return originVertex.containsNaN() || destinationVertex.containsNaN();
   }

   @Override
   public void setToNaN()
   {
      originVertex.setToNaN();
      destinationVertex.setToNaN();
   }

   @Override
   public void setToZero()
   {
      originVertex.setToZero();
      destinationVertex.setToZero();
   }

   public void clear()
   {
      setTwinHalfEdge(null);
      setOriginVertex(null);
      setDestinationVertex(null);
      setNextHalfEdge(null);
      setPreviousHalfEdge(null);
      setFace(null);
   }

   public void reverseEdge()
   {
      T newDestinationVertex = this.originVertex;
      setOriginVertex(destinationVertex);
      setDestinationVertex(newDestinationVertex);
      S newNextHalfEdge = this.previousHalfEdge;
      setPreviousHalfEdgeUnsafe(nextHalfEdge);
      setNextHalfEdgeUnsafe(newNextHalfEdge);
   }

   public String toString()
   {
      return "From: " + ((originVertex == null) ? "null" : originVertex.toString()) + ", To: "
            + ((destinationVertex == null) ? "null" : destinationVertex.toString());
   }

   @Override
   public double getX()
   {
      return getEdgeVector().getX();
   }

   @Override
   public double getY()
   {
      return getEdgeVector().getY();
   }

   @Override
   public double getZ()
   {
      return getEdgeVector().getZ();
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(point, this.originVertex, this.destinationVertex);
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
      if (percentage <= 0.0)
         this.originVertex.getSupportVectorDirectionTo(point, supportVectorToPack);
      else if (percentage >= 1.0)
         this.destinationVertex.getSupportVectorDirectionTo(point, supportVectorToPack);
      else
      {
         tempPoint.interpolate(this.originVertex, this.destinationVertex, percentage);
         supportVectorToPack.sub(point, tempPoint);
      }
   }

   @Override
   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
      if (percentage <= 0.0)
         this.originVertex.getSupportVectorJacobianTo(point, jacobianToPack);
      else if (percentage >= 1.0)
         this.destinationVertex.getSupportVectorJacobianTo(point, jacobianToPack);
      else
      {
         this.originVertex.getSupportVectorJacobianTo(point, jacobian);
         CommonOps.scale((1.0 - percentage), jacobian, jacobianToPack);
         this.destinationVertex.getSupportVectorJacobianTo(point, jacobian);
         CommonOps.addEquals(jacobianToPack, percentage, jacobian);
      }
   }

   public abstract S getThis();

   //   @Override
   //   public Q getSmallestSimplexMemberReference(Point3DReadOnly point)
   //   {
   //      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
   //      if (percentage <= 0.0)
   //         return this.originVertex;
   //      else if (percentage >= 1.0)
   //         return this.destinationVertex;
   //      else
   //         return this;
   //   }
}

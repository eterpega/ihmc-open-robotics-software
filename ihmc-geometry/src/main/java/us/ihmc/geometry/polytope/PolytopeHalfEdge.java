package us.ihmc.geometry.polytope;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class implements a doubly connected edge list (https://en.wikipedia.org/wiki/Doubly_connected_edge_list)
 * for storing polytope information
 * A half edge is completely described by its origin, destination and twin edge
 * The face, previous half edge and next half edge are stored for readability of code and should not be used for any geometrical operations
 * An attempt is made to update the twin in case the edge is modified to ensure that the relation remains consistent
 * @author Apoorv S
 */
public class PolytopeHalfEdge implements GeometryObject<PolytopeHalfEdge>, PolytopeHalfEdgeBasics
{
   private PolytopeHalfEdge twinEdge;
   private PolytopeHalfEdge nextHalfEdge;
   private PolytopeHalfEdge previousHalfEdge;
   private ConvexPolytopeFace face;
   private ExtendedPolytopeVertex originVertex;
   private ExtendedPolytopeVertex destinationVertex;
   /**
    * Not recomputed on change of values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();
   private Point3D tempPoint = new Point3D();
   private DenseMatrix64F jacobian = new DenseMatrix64F();
   /**
    * Default constructor. Does not initialize anything
    */
   public PolytopeHalfEdge()
   {

   }

   /**
    * Constructor for primary half edge
    * @param originVertex
    * @param destinationVertex
    */
   public PolytopeHalfEdge(ExtendedPolytopeVertex originVertex, ExtendedPolytopeVertex destinationVertex)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
   }

   public PolytopeHalfEdge(PolytopeHalfEdge edge)
   {
      set(edge);
   }

   /**
    * Function for creating the twin edge. Generates garbage. Use {@code getTwinHalfEdge()} to get the twin half edge
    * @return
    */
   public PolytopeHalfEdge createTwinHalfEdge()
   {
      return createTwinHalfEdge(null);
   }

   
   public PolytopeHalfEdge createTwinHalfEdge(ConvexPolytopeFace twinEdgeFace)
   {
      return new PolytopeHalfEdge(this, face);
   }

   /**
    * Function for creating the twin edge and storing its value. Generates garbage. Use {@code getTwinHalfEdge()} to get the twin half edge
    * @return
    */
   public PolytopeHalfEdge setAndCreateTwinHalfEdge(ConvexPolytopeFace face)
   {
      this.twinEdge = createTwinHalfEdge(face);
      return this.twinEdge;
   }

   public PolytopeHalfEdge setAndCreateTwinHalfEdge()
   {
      return setAndCreateTwinHalfEdge(null);
   }
   
   public void setToTwinOf(PolytopeHalfEdge twinEdge)
   {
      twinEdge.clear();
      twinEdge.setOriginVertex(this.destinationVertex);
      twinEdge.setDestinationVertex(this.originVertex);
      twinEdge.setTwinHalfEdge(this);
   }

   public PolytopeHalfEdge(PolytopeHalfEdge twinEdge, ConvexPolytopeFace face)
   {
      setTwinHalfEdge(twinEdge);
      setOriginVertex(twinEdge.getDestinationVertex());
      setDestinationVertex(twinEdge.getOriginVertex());
      setFace(face);
   }

   public PolytopeHalfEdge(ExtendedPolytopeVertex originVertex, ExtendedPolytopeVertex destinationVertex, PolytopeHalfEdge twinEdge, PolytopeHalfEdge nextHalfEdge,
                           PolytopeHalfEdge previousHalfEdge, ConvexPolytopeFace face)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
      setFace(face);
   }

   public void setOriginVertex(ExtendedPolytopeVertex originVertex)
   {
      if (this.originVertex != null)
         this.originVertex.removeAssociatedEdge(this);
      this.originVertex = originVertex;
      if (this.originVertex != null)
         this.originVertex.addAssociatedEdge(this);
      updateTwinDestination();
   }

   private void updateTwinOrigin()
   {
      if (twinEdge != null)
      {
         if (twinEdge.originVertex != null)
            twinEdge.originVertex.removeAssociatedEdge(twinEdge);
         twinEdge.originVertex = this.destinationVertex;
         if (twinEdge.originVertex != null)
            twinEdge.originVertex.addAssociatedEdge(twinEdge);
      }
   }

   private void updateTwinDestination()
   {
      if (twinEdge != null)
         twinEdge.destinationVertex = this.originVertex;
   }

   public ExtendedPolytopeVertex getOriginVertex()
   {
      return originVertex;
   }

   public void setDestinationVertex(ExtendedPolytopeVertex destinationVertex)
   {
      this.destinationVertex = destinationVertex;
      updateTwinOrigin();
   }

   public ExtendedPolytopeVertex getDestinationVertex()
   {
      return destinationVertex;
   }

   public void setTwinHalfEdge(PolytopeHalfEdge twinEdge)
   {
      this.twinEdge = twinEdge;
   }

   public PolytopeHalfEdge getTwinHalfEdge()
   {
      return twinEdge;
   }

   public void setNextHalfEdge(PolytopeHalfEdge nextHalfEdge)
   {
      if (nextHalfEdge == null || (nextHalfEdge.getOriginVertex() == this.getDestinationVertex() && nextHalfEdge.getFace() == this.getFace()))
         setNextHalfEdgeUnsafe(nextHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestinationVertex().toString() + " , next edge origin vertex: "
               + nextHalfEdge.getOriginVertex().toString());
   }

   private void setNextHalfEdgeUnsafe(PolytopeHalfEdge nextHalfEdge)
   {
      this.nextHalfEdge = nextHalfEdge;
   }

   public PolytopeHalfEdge getNextHalfEdge()
   {
      return nextHalfEdge;
   }

   public void setPreviousHalfEdge(PolytopeHalfEdge previousHalfEdge)
   {
      if (previousHalfEdge == null || (previousHalfEdge.getDestinationVertex() == this.getOriginVertex() && previousHalfEdge.getFace() == this.getFace()))
         setPreviousHalfEdgeUnsafe(previousHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOriginVertex().toString() + " , previous edge destination vertex: "
               + previousHalfEdge.getDestinationVertex().toString());
   }

   private void setPreviousHalfEdgeUnsafe(PolytopeHalfEdge previousHalfEdge)
   {
      this.previousHalfEdge = previousHalfEdge;
   }

   public PolytopeHalfEdge getPreviousHalfEdge()
   {
      return previousHalfEdge;
   }

   public void setFace(ConvexPolytopeFace face)
   {
      this.face = face;
   }

   public ConvexPolytopeFace getFace()
   {
      return face;
   }

   public Vector3D getEdgeVector()
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
   public boolean epsilonEquals(PolytopeHalfEdge other, double epsilon)
   {
      return getOriginVertex().epsilonEquals(other.getOriginVertex(), epsilon) && getDestinationVertex().epsilonEquals(other.getDestinationVertex(), epsilon);
   }

   public boolean isTwin(PolytopeHalfEdge twinEdge, double epsilon)
   {
      return epsilonEquals(twinEdge.getTwinHalfEdge(), epsilon);
   }

   @Override
   public void set(PolytopeHalfEdge other)
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
      ExtendedPolytopeVertex newDestinationVertex = this.originVertex;
      setOriginVertex(destinationVertex);
      setDestinationVertex(newDestinationVertex);
      PolytopeHalfEdge newNextHalfEdge = this.previousHalfEdge;
      setPreviousHalfEdgeUnsafe(nextHalfEdge);
      setNextHalfEdgeUnsafe(newNextHalfEdge);
   }

   public String toString()
   {
      return "From: " + ((originVertex == null)? "null" : originVertex.toString()) + ", To: " + ((destinationVertex == null)? "null" : destinationVertex.toString());
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
      if(percentage <= 0.0)
         this.originVertex.getSupportVectorDirectionTo(point, supportVectorToPack);
      else if(percentage >= 1.0)
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
      if(percentage <= 0.0)
         this.originVertex.getSupportVectorJacobianTo(point, jacobianToPack);
      else if(percentage >= 1.0)
         this.destinationVertex.getSupportVectorJacobianTo(point, jacobianToPack);
      else
      {
         this.originVertex.getSupportVectorJacobianTo(point, jacobian);
         CommonOps.scale((1.0 - percentage), jacobian, jacobianToPack);
         this.destinationVertex.getSupportVectorJacobianTo(point, jacobian);
         CommonOps.addEquals(jacobianToPack, percentage, jacobian);
      }
   }

   @Override
   public Simplex getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
      if(percentage <= 0.0)
         return this.originVertex;
      else if(percentage >= 1.0)
         return this.destinationVertex;
      else
         return this;
   }

   @Override
   public boolean geometricallyEquals(PolytopeHalfEdge other, double epsilon)
   {
      throw new RuntimeException("Not implemented.");
   }
}

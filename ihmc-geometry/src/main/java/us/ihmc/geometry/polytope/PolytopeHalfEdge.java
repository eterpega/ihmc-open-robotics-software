package us.ihmc.geometry.polytope;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * This class implements a doubly connected edge list (https://en.wikipedia.org/wiki/Doubly_connected_edge_list)
 * for storing polytope information
 * A half edge is completely described by its origin, destination and twin edge
 * The face, previous half edge and next half edge are stored for readability of code and should not be used for any geometrical operations
 * An attempt is made to update the twin in case the edge is modified to ensure that the relation remains consistent
 * @author Apoorv S
 */
public class PolytopeHalfEdge implements GeometryObject<PolytopeHalfEdge>
{
   private PolytopeHalfEdge twinEdge;
   private PolytopeHalfEdge nextHalfEdge;
   private PolytopeHalfEdge previousHalfEdge;
   private PolytopeFace face;
   private PolytopeVertex originVertex;
   private PolytopeVertex destinationVertex;
   /**
    * Not recomputed on change of values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();

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
   public PolytopeHalfEdge(PolytopeVertex originVertex, PolytopeVertex destinationVertex)
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

   
   public PolytopeHalfEdge createTwinHalfEdge(PolytopeFace twinEdgeFace)
   {
      return new PolytopeHalfEdge(this, face);
   }

   /**
    * Function for creating the twin edge and storing its value. Generates garbage. Use {@code getTwinHalfEdge()} to get the twin half edge
    * @return
    */
   public PolytopeHalfEdge setAndCreateTwinHalfEdge(PolytopeFace face)
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

   public PolytopeHalfEdge(PolytopeHalfEdge twinEdge, PolytopeFace face)
   {
      setTwinHalfEdge(twinEdge);
      setOriginVertex(twinEdge.getDestinationVertex());
      setDestinationVertex(twinEdge.getOriginVertex());
      setFace(face);
   }

   public PolytopeHalfEdge(PolytopeVertex originVertex, PolytopeVertex destinationVertex, PolytopeHalfEdge twinEdge, PolytopeHalfEdge nextHalfEdge,
                           PolytopeHalfEdge previousHalfEdge, PolytopeFace face)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
      setFace(face);
   }

   public void setOriginVertex(PolytopeVertex originVertex)
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

   public PolytopeVertex getOriginVertex()
   {
      return originVertex;
   }

   public void setDestinationVertex(PolytopeVertex destinationVertex)
   {
      this.destinationVertex = destinationVertex;
      updateTwinOrigin();
   }

   public PolytopeVertex getDestinationVertex()
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

   public void setFace(PolytopeFace face)
   {
      this.face = face;
   }

   public PolytopeFace getFace()
   {
      return face;
   }

   public Vector3D getEdgeVector()
   {
      edgeVector.sub(this.destinationVertex.getPosition(), this.originVertex.getPosition());
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
      PolytopeVertex newDestinationVertex = this.originVertex;
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

}

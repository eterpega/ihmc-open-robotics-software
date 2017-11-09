package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

/**
 * A template that defines the basic structure of a DCEL half edge. A half edge is composed of 
 * <li> {@code originVertex} starting point reference for this directed edge
 * <li> {@code destinatioVertex} ending point reference for this directed edge
 * <li> {@code twinHalfEdge} reference to the twin half edge on an adjacent face, if defined
 * <li> {@code nextHalfEdge} reference to the half edge on {@code face} that succeeds this edge in a counter clockwise sense
 * <li> {@code previousHalfEdge} reference to the half edge on {@code face} that precedes this edge in a counter clockwise sense 
 * <li> {@code face} the face that this half edge is a part of 
 * @author Apoorv S
 *
 * @param <V> Data structure representing a point in 3D space
 * @param <E> A class that extends this data structure Represents an directed edge formed by joining two vertices
 * @param <F> A collection of edges that constitute a face of the polytope
 */
public abstract class PolytopeHalfEdgeBasics<V extends PolytopeVertexBasics<V, E, F>, E extends PolytopeHalfEdgeBasics<V, E, F>, F extends ConvexPolytopeFaceBasics<V, E, F>>
      implements PolytopeHalfEdgeReadOnly, SimplexBasics, Clearable, Transformable, Settable<E>
{

   /**
    * Specifies the spatial location at which the half edge originates
    */
   private V originVertex;
   /**
    * Specifies the spatial location at which the half edge terminates
    */
   private V destinationVertex;
   /**
    * The half edge on an adjacent face that originates at the {@code destinatioVertex} and terminates at the 
    * {@code originVertex}. Represents the opposite spatial direction
    */
   private E twinEdge;
   /**
    * The half edge on the same face as this edge that originates at the {@code destinationVertex}
    */
   private E nextHalfEdge;
   /**
    * The half edge on the same face as this edge that terminates at the {@code originVertex}
    */
   private E previousHalfEdge;
   /**getShortestDistanceTo
    * The face that this edge is a part of 
    */
   private F face;
   /**
    * A vector that represents the direction and lenght of the half edge. 
    * Not recomputed on change of values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();
   /** 
    * A temporary variable for storing results
    */
   private Point3D tempPoint = new Point3D();

   /**
    * Returns a object of type {@code PolytopeHalfEdgeProvider} that can be used to generate half edges of the same 
    * type as this half edge 
    * @return an object that can be used to create other half edge objects
    */
   protected abstract PolytopeHalfEdgeProvider<V, E, F> getHalfEdgeProvider();

   /**
    * Default constructor 
    */
   public PolytopeHalfEdgeBasics()
   {

   }

   /**
    * Primary constructor for half edge
    * @param originVertex
    * @param destinationVertex
    */
   public PolytopeHalfEdgeBasics(V originVertex, V destinationVertex)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
   }

   /**
    * Copy constructor that copies all associations
    * @param edge
    */
   public PolytopeHalfEdgeBasics(E edge)
   {
      set(edge);
   }

   /**
    * 
    * @return a twin edge that can be used to generate a adjacent face. The twin edge generated stores references to 
    * the {@code originVertex} and {@code destinationVertex}. Half edge generated stores this edge as its twin but this 
    * half edge does not store the generated half edge as its twin
    */
   public E createTwinHalfEdge()
   {
      E twinEdge = getHalfEdgeProvider().getHalfEdge(getDestinationVertex(), getOriginVertex());
      twinEdge.setTwinHalfEdge((E) this);
      return twinEdge;
   }

   /**
    * Creates a half edge using the {@code createTwinHalfEdge{} function and stores a reference to the new object in the 
    * twin edge field
    * @return a twin edge
    */
   public E setAndCreateTwinHalfEdge()
   {
      E twinEdge = createTwinHalfEdge();
      setTwinHalfEdge(twinEdge);
      return twinEdge;
   }

   /**
    * Takes a edge, clears all its fields and assigns it all the values that a twin edge for this half edge would have 
    * i.e. {@code originVertex}, {@code destinationVertex}, {@code twinEdge = this}
    * @param twinEdge
    */
   public void setToTwin(E twinEdge)
   {
      twinEdge.clear();
      twinEdge.setOriginVertex(this.destinationVertex);
      twinEdge.setDestinationVertex(this.originVertex);
      twinEdge.setTwinHalfEdge((E) this);
   }

   /**
    * Creates a half edge from a twin edge and the face that the new half edge is to be a part of
    * @param twinEdge the edge that is to be the twin of the new half edgegetShortestDistanceTo
    * @param face the face that the new half edge is to be a part of
    */
   public PolytopeHalfEdgeBasics(E twinEdge, F face)
   {
      setTwinHalfEdge(twinEdge);
      setOriginVertex(twinEdge.getDestinationVertex());
      setDestinationVertex(twinEdge.getOriginVertex());
      setFace(face);
   }

   /**
    * Creates a half edge using all specified values 
    * @param originVertex the vertex that the new half edge will start at. Stored as a reference. Can be {@code null}
    * @param destinationVertex the vertex that the new half edge will end at. Stored as a reference. Can be {@code null}
    * @param twinEdge the half edge that is the DCEL twin of the new edge . Stored as a reference. Can be {@code null}
    * @param nextHalfEdge the half edge that is originates at the destination vertex and comes after the current edge when the face is traversed in a counter clockwise manner w.r.t. its face normal. Can be {@code null}
    * @param previousHalfEdge the half edge that is terminates at the origin vertex and comes before the current edge when the face is traversed in a counter clockwise manner w.r.t. its face normal. Can be {@code null}
    * @param face the face that this half edge is a part of. Can be {@code null}
    */
   public PolytopeHalfEdgeBasics(V originVertex, V destinationVertex, E twinEdge, E nextHalfEdge, E previousHalfEdge, F face)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
      setFace(face);
   }

   /**
    * Update the reference to the {@code originVertex} field to the specified value. Also updates the associated edges of the previously held and newly specified {@code originVertex} and the {@code twinEdge} of this edge
    * @param originVertex the new vertex that the half edge originates at. Can be null. Is modified
    */
   public void setOriginVertex(V originVertex)
   {
      if (this.originVertex != null)
         this.originVertex.removeAssociatedEdge((E) this);
      setOriginVertexUnsafe(originVertex);
      if (this.originVertex != null)
         this.originVertex.addAssociatedEdge((E) this);
      updateTwinDestination();
   }

   /**
    * Update the reference to the {@code originVertex} to the specified value. Associations are not updated
    * @param originVertex the new vertex that the half edge originates at. Can be null. Is not modified in this function
    */
   public void setOriginVertexUnsafe(V originVertex)
   {
      this.originVertex = originVertex;
   }

   /**
    * Internal function to update the origin of the twin edge only if the twin is not null. Is needed since the public versions that can set the origin would lead to a cyclic non-terminating call.
    * Also updates the requisite references. 
    */
   private void updateTwinOrigin()
   {
      if (twinEdge != null)
      {
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().removeAssociatedEdge(twinEdge);
         twinEdge.setOriginVertexUnsafe(this.destinationVertex);
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().addAssociatedEdge(twinEdge);
      }
   }

   /**
    * Internal function to update the destination vertex of the twin edge only if the twin is not null. Is needed since the public versions that can set the destination would lead to a cyclic non-terminating call.
    */
   private void updateTwinDestination()
   {
      if (twinEdge != null)
         twinEdge.setDestinationVertexUnsafe(this.originVertex);
   }

   /**
    * Returns a reference to the origin vertex for this half edge
    */
   public V getOriginVertex()
   {
      return originVertex;
   }

   /**
    * Update the reference to the {@code destinationVertex} to the specified value. Also updates the associated twin edge
    * @param destinationVertex the new vertex that the half edge originates at. Can be null. Is not modified in this function
    */
   public void setDestinationVertex(V destinationVertex)
   {
      this.destinationVertex = destinationVertex;
      updateTwinOrigin();
   }

   /**
    * Update the reference to the {@code destinationVertex} to the specified value. Associations of the specified vertex and of this object are not updated
    * @param destinationVertex the new vertex that the half edge originates at. Can be null. Is not modified in this function
    * @param destinationVertex
    */
   public void setDestinationVertexUnsafe(V destinationVertex)
   {
      this.destinationVertex = destinationVertex;
   }

   /**
    * Returns a reference to the {@code destinationVertex} of this half edge
    */
   public V getDestinationVertex()
   {
      return destinationVertex;
   }

   /**
    * Store a reference to the specified half edge as a twin of this half edge. No checks are performed while updating the twin edge.
    * @param twinEdge the half edge to be stored as a twin edge of this half edge. 
    */
   public void setTwinHalfEdge(E twinEdge)
   {
      this.twinEdge = twinEdge;
   }

   /**
    * {@inheritDoc}
    */
   public E getTwinHalfEdge()
   {
      return twinEdge;
   }

   /**
    * Update the reference to the {@code nextHalfEdge}. Checks to ensure that the origin of the specified edge and destination of the this half edge are the same
    * @param nextHalfEdge the new next half edge for the current half edge. Can be null
    * @throws RuntimeException in case the origin of this specified next half edge is not the same as the destination of the this edge
    */
   public void setNextHalfEdge(E nextHalfEdge)
   {
      if (nextHalfEdge == null || (nextHalfEdge.getOriginVertex() == this.getDestinationVertex() && nextHalfEdge.getFace() == this.getFace()))
         setNextHalfEdgeUnsafe(nextHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestinationVertex().toString() + " , next edge origin vertex: "
               + nextHalfEdge.getOriginVertex().toString());
   }
   
   /**
    * Internal method to update the next half edge without any checks
    * @param nextHalfEdge the half edge whose reference is to be stored in the next half edge field
    */
   private void setNextHalfEdgeUnsafe(E nextHalfEdge)
   {
      this.nextHalfEdge = nextHalfEdge;
   }

   /**
    * {@inheritDoc}
    */
   public E getNextHalfEdge()
   {
      return nextHalfEdge;
   }
   
   /**
    * Update the reference to the {@code previous HalfEdge}. Checks to ensure that the destination of the specified edge and origin of the this half edge are the same
    * @param previousHalfEdge the new previous half edge for the current half edge. Can be null
    * @throws RuntimeException in case the destination of this specified next half edge is not the same as the origin of the this edge
    */
   public void setPreviousHalfEdge(E previousHalfEdge)
   {
      if (previousHalfEdge == null || (previousHalfEdge.getDestinationVertex() == this.getOriginVertex() && previousHalfEdge.getFace() == this.getFace()))
         setPreviousHalfEdgeUnsafe(previousHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOriginVertex().toString() + " , previous edge destination vertex: "
               + previousHalfEdge.getDestinationVertex().toString());
   }

   /**
    * Internal method to update the next half edge without any checks
    * @param previousHalfEdge the half edge whose reference is to be stored in the previous half edge field
    */
   private void setPreviousHalfEdgeUnsafe(E previousHalfEdge)
   {
      this.previousHalfEdge = previousHalfEdge;
   }

   /**
    * {@inheritDoc}
    */
   public E getPreviousHalfEdge()
   {
      return previousHalfEdge;
   }

   /**
    * Update the reference to the face that this half edge is a part of
    * @param face the face reference to be stored. Can be null
    */
   public void setFace(F face)
   {
      this.face = face;
   }

   /**
    * {@inheritDoc}
    */
   public F getFace()
   {
      return face;
   }

   /**
    * {@inheritDoc}
    */
   public Vector3DReadOnly getEdgeVector()
   {
      edgeVector.sub(this.destinationVertex.getPosition(), this.originVertex.getPosition());
      return edgeVector;
   }

   /**
    * {@inheritDoc}
    */
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
   public boolean epsilonEquals(PolytopeHalfEdgeReadOnly other, double epsilon)
   {
      return getOriginVertex().epsilonEquals(other.getOriginVertex(), epsilon) && getDestinationVertex().epsilonEquals(other.getDestinationVertex(), epsilon);
   }

   /**
    * {@inheritDoc}
    */
   public boolean isTwin(PolytopeHalfEdgeReadOnly twinEdge, double epsilon)
   {
      return epsilonEquals(twinEdge.getTwinHalfEdge(), epsilon);
   }

   /**
    * Copies all the references from the specified half edge to this half edge while updating the associated objects 
    */
   public void set(E other)
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

   /**
    * {@inheritDoc}
    * Also sets the {@code originVertex} and {@code destinationVertex} to NaN
    */
   @Override
   public void setToNaN()
   {
      originVertex.setToNaN();
      destinationVertex.setToNaN();
   }

   /**
    * {@inheritDoc}
    * Also sets the {@code originVertex} and {@code destinationVertex} to zero
    */
   @Override
   public void setToZero()
   {
      originVertex.setToZero();
      destinationVertex.setToZero();
   }

   /**
    * Sets all the references that are held by this half edge to null and also updates the previously associated objects 
    */
   public void clear()
   {
      setTwinHalfEdge(null);
      setOriginVertex(null);
      setDestinationVertex(null);
      setNextHalfEdge(null);
      setPreviousHalfEdge(null);
      setFace(null);
   }

   /**
    * Changes the direction of this edge so that it starts at the previous {@code destinationVertex} and ends at the {@code originVertex}
    * The references to the {@code nextHalfEdge} and {@code previousHalfEdge} are also updated as its the twin edge.
    * Reference to the {@code face} is not changed
    */
   public void reverseEdge()
   {
      V newDestinationVertex = this.originVertex;
      setOriginVertex(destinationVertex);
      setDestinationVertex(newDestinationVertex);
      E newNextHalfEdge = this.previousHalfEdge;
      setPreviousHalfEdgeUnsafe(nextHalfEdge);
      setNextHalfEdgeUnsafe(newNextHalfEdge);
   }

   /**
    * {@inheritDoc}
    */
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
   public SimplexBasics getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(point, this.originVertex, this.destinationVertex);
      if (percentage <= 0.0)
         return this.originVertex;
      else if (percentage >= 1.0)
         return this.destinationVertex;
      else
         return this;
   }
}

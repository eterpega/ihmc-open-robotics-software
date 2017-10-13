package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

/**
 * This class defines a polytope face. A face is defined by the set of edges that bound it.
 * 
 * @author Apoorv S
 *
 */
public class ConvexPolytopeFace implements GeometryObject<ConvexPolytopeFace>, ConvexPolytopeFaceBasics
{
   private final double EPSILON = Epsilons.ONE_MILLIONTH;
   
   /**
    * Ordered list of half edges that bound the face
    */
   private final ArrayList<PolytopeHalfEdge> edges = new ArrayList<>();
   /**
    * Do not access directly since this is updated only when the getter is called
    */
   private final Vector3D faceNormal = new Vector3D();
   /**
    * Do not access directly since this is updated only when the getter is called
    */
   private final Point3D faceCentroid = new Point3D();
   
   // Temporary variables for calculations
   private final Vector3D tempVector = new Vector3D();
   private final Point3D tempPoint = new Point3D();
   private final ArrayList<PolytopeHalfEdge> visibleEdgeList = new ArrayList<>();
   private boolean marked = false;
   /**
    * Default constructor. Does not initialize anything
    */
   public ConvexPolytopeFace()
   {

   }

   public ConvexPolytopeFace(ConvexPolytopeFace other)
   {
      this(other.getEdgeList());
   }

   public ConvexPolytopeFace(List<PolytopeHalfEdge> edgeList)
   {
      this.copyEdgeList(edgeList);
   }

   public ConvexPolytopeFace(PolytopeHalfEdge[] edgeListArray)
   {
      this.copyEdgeList(edgeListArray);
   }

   public void copyEdgeList(PolytopeHalfEdge[] edgeListArray)
   {
      edges.clear();
      for (int i = 0; i < edgeListArray.length; i++)
      {
         this.edges.add(edgeListArray[i]);
      }
   }

   public void copyEdgeList(List<PolytopeHalfEdge> edgeList)
   {
      this.edges.clear();
      this.edges.addAll(edgeList);
   }

   public List<PolytopeHalfEdge> getEdgeList()
   {
      return edges;
   }
   
   public PolytopeHalfEdge getEdge(int index)
   {
      return edges.get(index);
   }

   public void addVertex(PolytopeVertex vertexToAdd)
   {
      switch (edges.size())
      {
      case 0:
      {
         PolytopeHalfEdge newEdge = new PolytopeHalfEdge(vertexToAdd, vertexToAdd);
         newEdge.setFace(this);
         newEdge.setNextHalfEdge(newEdge);
         newEdge.setPreviousHalfEdge(newEdge);
         edges.add(newEdge);
         break;
      }
      case 1:
      {
         // Set the edge for the two points and then create its twin
         edges.get(0).setDestinationVertex(vertexToAdd);
         PolytopeHalfEdge newEdge = new PolytopeHalfEdge(vertexToAdd, edges.get(0).getOriginVertex());
         newEdge.setFace(this);
         newEdge.setNextHalfEdge(edges.get(0));
         newEdge.setPreviousHalfEdge(edges.get(0));
         edges.get(0).setNextHalfEdge(newEdge);
         edges.get(0).setPreviousHalfEdge(newEdge);
         edges.add(newEdge);
         break;
      }
      case 2:
      {
         // Create a new edge and assign an arbitrary configuration since there is no way to tell up and down in 3D space
         edges.get(1).setDestinationVertex(vertexToAdd);
         PolytopeHalfEdge newEdge = new PolytopeHalfEdge(vertexToAdd, edges.get(0).getOriginVertex());
         newEdge.setFace(this);
         edges.add(newEdge);
         newEdge.setNextHalfEdge(edges.get(0));
         edges.get(0).setPreviousHalfEdge(newEdge);
         newEdge.setPreviousHalfEdge(edges.get(1));
         edges.get(1).setNextHalfEdge(newEdge);
         break;
      }
      default:
      {
         // Now a ordering is available and all new vertices to add must be done accordingly. Also points must lie in the same plane
         if(!isPointInFacePlane(vertexToAdd, EPSILON))
            return;
         
         getVisibleEdgeList(vertexToAdd, visibleEdgeList);
         switch (visibleEdgeList.size())
         {
         case 0: return; // Case where the point is internal
         case 1: 
            PolytopeHalfEdge additionalEdge = new PolytopeHalfEdge(vertexToAdd, visibleEdgeList.get(0).getDestinationVertex());
            additionalEdge.setFace(this);
            visibleEdgeList.get(0).setDestinationVertex(vertexToAdd);
            additionalEdge.setNextHalfEdge(visibleEdgeList.get(0).getNextHalfEdge());
            visibleEdgeList.get(0).getNextHalfEdge().setPreviousHalfEdge(additionalEdge);
            visibleEdgeList.get(0).setNextHalfEdge(additionalEdge);
            additionalEdge.setPreviousHalfEdge(visibleEdgeList.get(0));
            edges.add(additionalEdge);
            break;
         default:
            visibleEdgeList.get(0).setDestinationVertex(vertexToAdd);
            visibleEdgeList.get(visibleEdgeList.size() - 1).setOriginVertex(vertexToAdd);
            visibleEdgeList.get(0).setNextHalfEdge(visibleEdgeList.get(visibleEdgeList.size() - 1));
            visibleEdgeList.get(visibleEdgeList.size() - 1).setPreviousHalfEdge(visibleEdgeList.get(0));
            for(int i = 1; i < visibleEdgeList.size() - 1; i++)
               edges.remove(visibleEdgeList.get(i));
            break;
         }
         break;
      }
      }
   }
   
   /**
    * You have been given power. Do not abuse it (just ensure that the additions are all consistent)
    * @param newEdge
    */
   public void addEdge(PolytopeHalfEdge newEdge)
   {
      edges.add(newEdge);
   }

   public void getVisibleEdgeList(Point3DReadOnly vertex, List<PolytopeHalfEdge> edgeList)
   {
      edgeList.clear();
      PolytopeHalfEdge edgeUnderConsideration = getFirstVisibleEdge(vertex);
      for(int i = 0; edgeUnderConsideration != null && i < edges.size(); i++)
      {
         edgeList.add(edgeUnderConsideration);
         edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
         if(isPointOnInteriorSideOfEdgeInternal(vertex, edgeUnderConsideration))
            break;
      }
   }

   public PolytopeHalfEdge getFirstVisibleEdge(Point3DReadOnly vertex)
   {
      if(edges.size() == 0)
         return null;
      else if (edges.size() == 1 || edges.size() == 2)         
         return edges.get(0);
      
      PolytopeHalfEdge edgeUnderConsideration = edges.get(0);
      double previousDotProduct = Double.NaN;
      for(int i = 0 ; i < getNumberOfEdges(); i++)
      {
         double dotProduct = getEdgeVisibilityProduct(vertex, edgeUnderConsideration);
         if(dotProduct <= 0) 
            edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
         else
            edgeUnderConsideration = edgeUnderConsideration.getPreviousHalfEdge();
         
         if(previousDotProduct * dotProduct <= 0)
         {
            if(previousDotProduct < 0)
               edgeUnderConsideration = edgeUnderConsideration.getNextHalfEdge();
            return edgeUnderConsideration;
         }
         else
            previousDotProduct = dotProduct;
      }
      PrintTools.debug("Reaching here");
      return edgeUnderConsideration;
   }
   
   public boolean isPointOnInteriorSideOfEdgeInternal(Point3DBasics point, int index)
   {
      updateFaceNormal();
      return isPointOnInteriorSideOfEdgeInternal(point, edges.get(index));
   }
   
   private boolean isPointOnInteriorSideOfEdgeInternal(Point3DReadOnly point, PolytopeHalfEdge halfEdge)
   {
      return getEdgeVisibilityProduct(point, halfEdge) < 0; 
   }
   
   public double getFaceVisibilityProduct(Point3DReadOnly point)
   {
      tempVector.sub(point, getEdge(0).getOriginVertex());
      return dotFaceNormal(tempVector);
   }

   private double getEdgeVisibilityProduct(Point3DReadOnly point, PolytopeHalfEdge halfEdge)
   {
      tempVector.sub(point, halfEdge.getOriginVertex());
      tempVector.cross(halfEdge.getEdgeVector());
      return tempVector.dot(getFaceNormal());
   }
   
   public boolean isPointInFacePlane(Point3DReadOnly vertexToCheck, double epsilon)
   {
      tempVector.sub(vertexToCheck, edges.get(0).getOriginVertex());
      return MathTools.epsilonEquals(tempVector.dot(getFaceNormal()), 0.0, epsilon);
   }
   
   public boolean isInteriorPoint(Point3DReadOnly vertexToCheck)
   {
      return (isPointInFacePlane(vertexToCheck, EPSILON) && isInteriorPointInternal(vertexToCheck));
   }
   
   private boolean isInteriorPointInternal(Point3DReadOnly vertexToCheck)
   {
      if(edges.size() < 3)
         return false;
      
      boolean result = true;
      PolytopeHalfEdge halfEdge = edges.get(0);
      for(int i = 0; result && i < edges.size(); i++)
      {
         result &= isPointOnInteriorSideOfEdgeInternal(vertexToCheck, halfEdge); 
         halfEdge = halfEdge.getNextHalfEdge();
      }
      return result;
   }
   
   public Point3D getFaceCentroid()
   {
      updateFaceCentroid();
      return faceCentroid;
   }
   
   private void updateFaceCentroid()
   {
      faceCentroid.setToZero();
      for(int i = 0; i < edges.size(); i++)
         faceCentroid.add(edges.get(i).getOriginVertex());
      faceCentroid.scale(1.0 / edges.size());
   }
   
   public Vector3D getFaceNormal()
   {
      updateFaceNormal();
      return faceNormal;
   }

   public Vector3D getNormailizedFaceNormal()
   {
      updateFaceNormal();
      faceNormal.normalize();
      return faceNormal;
   }
   
   private void updateFaceNormal()
   {
      if(edges.size() < 3)
         faceNormal.setToZero();
      else
      {
         faceNormal.cross(edges.get(0).getEdgeVector(), edges.get(0).getNextHalfEdge().getEdgeVector());
         faceNormal.normalize();
      }
   }
   
   public int getNumberOfEdges()
   {
      return edges.size();
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOriginVertex().applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).getOriginVertex().applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(ConvexPolytopeFace other, double epsilon)
   {
      if(other.getNumberOfEdges() == this.getNumberOfEdges())
      {
         int index = findMatchingEdgeIndex(other.getEdge(0), epsilon);
         if(index !=-1)
         {
            boolean result = true;
            PolytopeHalfEdge matchedEdge = edges.get(index);
            PolytopeHalfEdge candidateEdge = other.getEdge(0);
            for(int i = 0; result && i < edges.size() - 1; i++)
            {
               matchedEdge = matchedEdge.getNextHalfEdge();
               candidateEdge = candidateEdge.getNextHalfEdge();
               result &= matchedEdge.epsilonEquals(candidateEdge, epsilon);
            }
            return result;
         }
         else
            return false;
      }
      else 
         return false;
   }

   public int findMatchingEdgeIndex(PolytopeHalfEdge edgeToSearch, double epsilon)
   {
      for(int i = 0; i < edges.size(); i++)
      {
         if(edges.get(i).epsilonEquals(edgeToSearch, epsilon))
            return i;
      }
      return -1;
   }
   
   public PolytopeHalfEdge findMatchingEdge(PolytopeHalfEdge edgeToSearch, double epsilon)
   {
      return edges.get(findMatchingEdgeIndex(edgeToSearch, epsilon));
   }
   
   public void reverseFaceNormal()
   {
      for(int i = 0; i < edges.size(); i++)
      {
         edges.get(i).reverseEdge();
      }
      updateFaceNormal();
   }
   
   @Override
   public void set(ConvexPolytopeFace other)
   {
      clearEdgeList();
      copyEdgeList(other.edges);
   }

   public void clearEdgeList()
   {
      this.edges.clear();
   }

   @Override
   public boolean containsNaN()
   {
      boolean result = (edges.size() > 0 && edges.get(0).containsNaN());
      for (int i = 1; !result && i < edges.size(); i++)
         result |= edges.get(i).getDestinationVertex().containsNaN();
      return result;
   }

   @Override
   public void setToNaN()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToNaN();
   }
   
   public double dotFaceNormal(Vector3DBasics direction)
   {
      updateFaceNormal();
      return direction.dot(faceNormal);
   }
   
   public boolean isFaceVisible(Point3DReadOnly point, double epsilon)
   {
      return getFaceVisibilityProduct(point) > epsilon;
   }

   @Override
   public void setToZero()
   {
      for (int i = 0; i < edges.size(); i++)
         edges.get(i).setToZero();
   }
   
   public double getMaxElement(int index)
   {
      PolytopeHalfEdge edgeReference = edges.get(0);
      double maxElement = edgeReference.getOriginVertex().getElement(index);
      for(int i = 0; i < edges.size(); i++)
      {
         if(maxElement < edgeReference.getDestinationVertex().getElement(index))
            maxElement = edgeReference.getDestinationVertex().getElement(index);
         edgeReference = edgeReference.getNextHalfEdge();
      }
      return maxElement;
   }

   public double getMinElement(int index)
   {
      PolytopeHalfEdge edgeReference = edges.get(0);
      double minElement = edgeReference.getOriginVertex().getElement(index);
      for(int i = 0; i < edges.size(); i++)
      {
         if(minElement > edgeReference.getDestinationVertex().getElement(index))
            minElement = edgeReference.getDestinationVertex().getElement(index);
         edgeReference = edgeReference.getNextHalfEdge();
      }
      return minElement;
   }
   
   public double getMaxX()
   {
      return getMaxElement(0);
   }

   public double getMaxY()
   {
      return getMaxElement(1);
   }

   public double getMaxZ()
   {
      return getMaxElement(2);
   }

   public double getMinX()
   {
      return getMinElement(0);
   }

   public double getMinY()
   {
      return getMinElement(1);
   }

   public double getMinZ()
   {
      return getMinElement(2);
   }
   
   public ConvexPolytopeFace getNeighbouringFace(int index)
   {
      if(index > edges.size() || edges.get(index).getTwinHalfEdge() == null)
         return null;
      else
         return edges.get(index).getTwinHalfEdge().getFace();
               
   }
   
   public Point3D getSupportingVertex(Vector3D supportVector)
   {
      PolytopeVertex bestVertex = edges.get(0).getOriginVertex();
      double maxDot = bestVertex.dot(supportVector);
      PolytopeVertex bestVertexCandidate = bestVertex;
      while(true)
      {
         bestVertexCandidate = bestVertex;
         for(int i = 0; i < bestVertex.getNumberOfAssociatedEdges(); i++)
         {
            double dotCandidate = bestVertex.getAssociatedEdges().get(i).getDestinationVertex().dot(supportVector);
            if(maxDot < dotCandidate)
            {
               maxDot = dotCandidate;
               bestVertexCandidate = bestVertex.getAssociatedEdge(i).getDestinationVertex();
            }
         }
         if(bestVertexCandidate == bestVertex)
            return bestVertex.getPosition();
         else
            bestVertex = bestVertexCandidate;
      }
   }
   
   public void mark()
   {
      this.marked = true;
   }
   
   public void unmark()
   {
      this.marked = false;
   }
   
   public boolean isMarked()
   {
      return this.marked;
   }
   
   public boolean isNotMarked()
   {
      return !this.marked;
   }
   
   @Override
   public String toString()
   {
      String string = "";
      PolytopeHalfEdge edge = edges.get(0);
      for(int i = 0; i < edges.size(); i++)
      {
         string += "\n" + edge.toString() + " Twin: " + (edge.getTwinHalfEdge() == null ? "null" : edge.getTwinHalfEdge().toString());
         edge = edge.getNextHalfEdge();
      }
      return string;
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getNormailizedFaceNormal(), tempPoint);
      if(isInteriorPointInternal(tempPoint))
         return point.distance(tempPoint);
      else
         return getEdgeClosestTo(tempPoint).getShortestDistanceTo(point);
   }
   
   public PolytopeHalfEdge getEdgeClosestTo(Point3DReadOnly point)
   {
      PolytopeHalfEdge edge = getFirstVisibleEdge(tempPoint);
      double shortestDistance = edge.getShortestDistanceTo(tempPoint);
      double shortestDistanceCandidate = Double.NEGATIVE_INFINITY;
      while(shortestDistanceCandidate <= shortestDistance)
      {
         edge = edge.getNextHalfEdge();
         shortestDistanceCandidate = edge.getShortestDistanceTo(tempPoint);
      }
      return edge.getPreviousHalfEdge();
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getNormailizedFaceNormal(), tempPoint);
      if(isInteriorPointInternal(tempPoint))
         supportVectorToPack.set(getFaceNormal());
      else
         getEdgeClosestTo(tempPoint).getSupportVectorDirectionTo(point, supportVectorToPack);
   }

   @Override
   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getNormailizedFaceNormal(), tempPoint);
      if(isInteriorPointInternal(tempPoint))
      {
         throw new RuntimeException("Unimplemented case");
      }
      else
         getEdgeClosestTo(tempPoint).getSupportVectorJacobianTo(point, jacobianToPack);
   }

   @Override
   public Simplex getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      EuclidGeometryTools.orthogonalProjectionOnPlane3D(point, edges.get(0).getOriginVertex(), getNormailizedFaceNormal(), tempPoint);
      if(isInteriorPointInternal(tempPoint))
         return this;
      else
         return getEdgeClosestTo(tempPoint).getSmallestSimplexMemberReference(point);
   }
}

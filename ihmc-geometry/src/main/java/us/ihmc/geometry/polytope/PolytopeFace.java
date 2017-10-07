package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

/**
 * This class defines a polytope face. A face is defined by the set of edges that bound it.
 * 
 * @author Apoorv S
 *
 */
public class PolytopeFace implements GeometryObject<PolytopeFace>
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

   /**
    * Default constructor. Does not initialize anything
    */
   public PolytopeFace()
   {

   }

   public PolytopeFace(PolytopeFace other)
   {
      this(other.getEdgeList());
   }

   public PolytopeFace(List<PolytopeHalfEdge> edgeList)
   {
      this.copyEdgeList(edgeList);
   }

   public void copyEdgeList(List<PolytopeHalfEdge> edgeList)
   {
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
         // Create a fake edge of zero length and assign to the face
         PolytopeHalfEdge newEdge = new PolytopeHalfEdge(vertexToAdd, vertexToAdd);
         newEdge.setFace(this);
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
         updateFaceNormal();
         PolytopeVertex vertexCandidate = null;
         PolytopeHalfEdge newHalfEdgeCandidate = edges.get(0);
         
         // Figure out if this point is an internal point. If it is not then which edge should be expanded to include it 
         int index = 0;
         for(; index < edges.size(); index++)
         {
            tempVector.sub(vertexToAdd, newHalfEdgeCandidate.getOriginVertex());
            tempVector.cross(newHalfEdgeCandidate.getEdgeVector());
            if(tempVector.dot(faceNormal) > 0)
            {
               vertexCandidate = newHalfEdgeCandidate.getDestinationVertex();
               newHalfEdgeCandidate.setDestinationVertex(vertexToAdd);
               break;
            }
            newHalfEdgeCandidate = newHalfEdgeCandidate.getNextHalfEdge();
         }
         
         // handle the case that the point is an interior point 
         if(vertexCandidate == null)
            return;
         
         // Begin searching for the new connections that must be made
         PolytopeHalfEdge newHalfEdge = newHalfEdgeCandidate;
         newHalfEdgeCandidate = newHalfEdgeCandidate.getNextHalfEdge();
         
         // Handle the one case in which a new edge must be created here to minimize garbage
         tempVector.sub(vertexCandidate, vertexToAdd);
         tempVector.cross(newHalfEdgeCandidate.getEdgeVector());
         if(tempVector.dot(faceNormal) > 0)
         {
            PolytopeHalfEdge additionalEdge = new PolytopeHalfEdge(vertexToAdd, vertexCandidate);
            additionalEdge.setFace(this);
            additionalEdge.setNextHalfEdge(newHalfEdge.getNextHalfEdge());
            newHalfEdge.getNextHalfEdge().setPreviousHalfEdge(additionalEdge);
            newHalfEdge.setNextHalfEdge(additionalEdge);
            additionalEdge.setPreviousHalfEdge(newHalfEdge);
            edges.add(index + 1, additionalEdge);
         }
         // Handle the standard case where edges must be removed / re-purposed to add this point
         else
         {
            // If this loop does not break then something is very wrong
            while(true)
            {
               vertexCandidate = newHalfEdgeCandidate.getDestinationVertex();
               tempVector.sub(vertexCandidate, vertexToAdd);
               tempVector.cross(newHalfEdgeCandidate.getNextHalfEdge().getEdgeVector());
               if(tempVector.dot(faceNormal) > 0)
               {
                  newHalfEdgeCandidate.setOriginVertex(vertexToAdd);
                  newHalfEdgeCandidate.setPreviousHalfEdge(newHalfEdge);
                  newHalfEdge.setNextHalfEdge(newHalfEdgeCandidate);
                  break;
               }
               newHalfEdgeCandidate = newHalfEdgeCandidate.getNextHalfEdge();
               edges.remove(newHalfEdgeCandidate.getPreviousHalfEdge());
            }
         }
         break;
      }
      }
   }

   public boolean isPointInFacePlane(Point3DBasics vertexToCheck, double epsilon)
   {
      updateFaceNormal();
      tempVector.sub(vertexToCheck, edges.get(0).getOriginVertex());
      return MathTools.epsilonEquals(tempVector.dot(faceNormal), 0.0, epsilon);
   }
   
   public boolean isInteriorPoint(PolytopeVertex vertexToCheck)
   {
      return (isPointInFacePlane(vertexToCheck, EPSILON) && isInteriorPointInternal(vertexToCheck));
   }
   
   private boolean isInteriorPointInternal(PolytopeVertex vertexToCheck)
   {
      // Handle the case where the face is ill - defined 
      if(edges.size() < 3)
         return false;
      
      updateFaceNormal();
      
      boolean result = true;
      for(int i = 0; result && i < edges.size(); i++)
      {
         tempVector.sub(vertexToCheck, edges.get(i).getOriginVertex());
         tempVector.cross(edges.get(i).getEdgeVector());
         result &= (tempVector.dot(faceNormal) < 0); 
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
         edges.get(i).applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < getNumberOfEdges(); i++)
         edges.get(i).applyInverseTransform(transform);
   }

   @Override
   public boolean epsilonEquals(PolytopeFace other, double epsilon)
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
   public void set(PolytopeFace other)
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
   
   public boolean isFaceVisible(Point3DBasics point)
   {
      tempVector.sub(point, edges.get(0).getOriginVertex());
      return dotFaceNormal(tempVector) > 0;
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
   
   public PolytopeFace getNeighbouringFace(int index)
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
               bestVertexCandidate = bestVertex.getAssociatedEdges().get(i).getDestinationVertex();
            }
         }
         if(bestVertexCandidate == bestVertex)
            return bestVertex.getPosition();
         else
            bestVertex = bestVertexCandidate;
      }
   }
   
   @Override
   public String toString()
   {
      String string = "";
      PolytopeHalfEdge edge = edges.get(0);
      for(int i = 0; i < edges.size(); i++)
      {
         string += "\n" + edge.toString();
         edge = edge.getNextHalfEdge();
      }
      return string;
   }
}

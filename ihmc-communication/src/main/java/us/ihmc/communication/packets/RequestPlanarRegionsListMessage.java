package us.ihmc.communication.packets;

import us.ihmc.euclid.geometry.BoundingBox3D;

public class RequestPlanarRegionsListMessage extends Packet<RequestPlanarRegionsListMessage>
{
   public enum RequestType {SINGLE_UPDATE, CONTINUOUS_UPDATE, STOP_UPDATE, CLEAR};

   public RequestType requestType;
   public BoundingBox3D boundingBoxInWorldForRequest;

   public RequestPlanarRegionsListMessage()
   {
   }

   public RequestPlanarRegionsListMessage(RequestType requestType)
   {
      this(requestType, null, null);
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, BoundingBox3D boundingBoxInWorldForRequest)
   {
      this(requestType, boundingBoxInWorldForRequest, null);
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, PacketDestination destination)
   {
      this(requestType, null, destination);
   }

   public RequestPlanarRegionsListMessage(RequestType requestType, BoundingBox3D boundingBoxInWorldForRequest, PacketDestination destination)
   {
      this.requestType = requestType;
      this.boundingBoxInWorldForRequest = boundingBoxInWorldForRequest;
      if (destination != null)
         setDestination(destination);
   }

   public void set(RequestPlanarRegionsListMessage other)
   {
      this.requestType = other.requestType;
      this.boundingBoxInWorldForRequest = other.boundingBoxInWorldForRequest;
      setDestination(other.getDestination());
   }

   public RequestType getRequesType()
   {
      return requestType;
   }

   public void setRequesType(RequestType requesType)
   {
      this.requestType = requesType;
   }

   public boolean hasBoundingBox()
   {
      return boundingBoxInWorldForRequest != null;
   }

   public BoundingBox3D getBoundingBoxInWorldForRequest()
   {
      return boundingBoxInWorldForRequest;
   }

   @Override
   public boolean epsilonEquals(RequestPlanarRegionsListMessage other, double epsilon)
   {
      boolean typeEqual = requestType == other.requestType;
      boolean boxEqual;
      if (hasBoundingBox() && other.hasBoundingBox())
         boxEqual = boundingBoxInWorldForRequest.epsilonEquals(other.boundingBoxInWorldForRequest, epsilon);
      else if (!hasBoundingBox() && !other.hasBoundingBox())
         boxEqual = true;
      else
         boxEqual = false;
      return typeEqual && boxEqual;
   }
}

package us.ihmc.exampleSimulations.rolling;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class MobileRobotDescription extends RobotDescription
{
   private int collisionGroup = 0xffffffff;
   private int collisionMask = 0xffffffff;

   private double massBody = 1.0;
   private double xLengthBody = 0.8;
   private double yLengthBody = 0.4;
   private double zLengthBody = 0.1;

   private double massWheel = 0.5;
   private double wheelPlacementRatio = 0.8; // ratio with length of x from center to the end in x direction
   private double radiusWheel = 0.08;
   private double lengthWheel = yLengthBody * 0.8;

   public MobileRobotDescription(String name)
   {
      super(name);

      FloatingJointDescription bodyJoint = new FloatingJointDescription("body", "bodyjointvariablename");

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(massBody, xLengthBody, yLengthBody, zLengthBody);

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -zLengthBody / 2.0);
      bodyLinkGraphics.addCube(xLengthBody, yLengthBody, zLengthBody, YoAppearance.Red());
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      CollisionMeshDescription bodyCollisionMesh = new CollisionMeshDescription();
      bodyCollisionMesh.addCubeReferencedAtCenter(xLengthBody, yLengthBody, zLengthBody);
      bodyCollisionMesh.setCollisionGroup(collisionGroup);
      bodyCollisionMesh.setCollisionMask(collisionMask);
      bodyLink.addCollisionMesh(bodyCollisionMesh);

      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      PinJointDescription frontWheelJoint = new PinJointDescription("frontwheel", new Vector3D(xLengthBody / 2.0 * wheelPlacementRatio, 0.0,
                                                                                               -zLengthBody / 2.0 - radiusWheel),
                                                                    Axis.Y);
      LinkDescription frontWheelLink = new LinkDescription("frontWheelLink");
      frontWheelLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, 0.0));
      frontWheelLink.setMassAndRadiiOfGyration(massWheel, radiusWheel, radiusWheel, lengthWheel);

      LinkGraphicsDescription frontWheelGraphics = new LinkGraphicsDescription();
      AppearanceDefinition frontWheelAppearance = YoAppearance.Black();
      frontWheelGraphics.rotate(Math.PI / 2.0, Axis.X);
      frontWheelGraphics.translate(new Vector3D(0.0, 0.0, -lengthWheel / 2.0));
      frontWheelGraphics.addCylinder(lengthWheel, radiusWheel, frontWheelAppearance);
      frontWheelLink.setLinkGraphics(frontWheelGraphics);

      CollisionMeshDescription frontWheelCollisionMesh = new CollisionMeshDescription();
      frontWheelCollisionMesh.rotate(Math.PI / 2.0, Axis.X);
      frontWheelCollisionMesh.translate(new Vector3D(0.0, 0.0, -lengthWheel / 2.0));
      frontWheelCollisionMesh.addCylinderReferencedAtBottomMiddle(radiusWheel, lengthWheel);
      frontWheelCollisionMesh.setCollisionGroup(collisionGroup);
      frontWheelCollisionMesh.setCollisionMask(collisionMask);
      frontWheelLink.addCollisionMesh(frontWheelCollisionMesh);

      frontWheelJoint.setLink(frontWheelLink);
      bodyJoint.addJoint(frontWheelJoint);

      PinJointDescription rearWheelJoint = new PinJointDescription("rearwheel", new Vector3D(-xLengthBody / 2.0 * wheelPlacementRatio, 0.0,
                                                                                             -zLengthBody / 2.0 - radiusWheel),
                                                                   Axis.Y);
      LinkDescription rearWheelLink = new LinkDescription("rearWheelLink");
      rearWheelLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, 0.0));
      rearWheelLink.setMassAndRadiiOfGyration(massWheel, radiusWheel, radiusWheel, lengthWheel);

      LinkGraphicsDescription rearWheelGraphics = new LinkGraphicsDescription();
      AppearanceDefinition rearWheelAppearance = YoAppearance.Grey();
      rearWheelGraphics.rotate(Math.PI / 2.0, Axis.X);
      rearWheelGraphics.translate(new Vector3D(0.0, 0.0, -lengthWheel / 2.0));
      rearWheelGraphics.addCylinder(lengthWheel, radiusWheel, rearWheelAppearance);
      rearWheelLink.setLinkGraphics(rearWheelGraphics);

      CollisionMeshDescription rearWheelCollisionMesh = new CollisionMeshDescription();
      rearWheelCollisionMesh.rotate(Math.PI / 2.0, Axis.X);
      rearWheelCollisionMesh.translate(new Vector3D(0.0, 0.0, -lengthWheel / 2.0));
      rearWheelCollisionMesh.addCylinderReferencedAtBottomMiddle(radiusWheel, lengthWheel);
      rearWheelCollisionMesh.setCollisionGroup(collisionGroup);
      rearWheelCollisionMesh.setCollisionMask(collisionMask);
      rearWheelLink.addCollisionMesh(rearWheelCollisionMesh);

      rearWheelJoint.setLink(rearWheelLink);
      bodyJoint.addJoint(rearWheelJoint);
   }

}

package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;

public class GroundRobot
{
   //   private final Link baseLink;

   private int estimatedNumberOfContactPoints = 40;
   private double groundAngle = 0.0;
   private boolean addWalls = false;
   private int collisionGroup = 0xffffffff;
   private int collisionMask = 0xffffffff;

   private double groundLength = 4.0;
   private double groundWidth = 4.0;
   private double groundThickness = 0.05;

   public GroundRobot()
   {
   }

   public void setGroundLength(double groundLength)
   {
      this.groundLength = groundLength;
   }

   public void setGroundWidth(double groundWidth)
   {
      this.groundWidth = groundWidth;
   }

   public void setGroundThickness(double groundThickness)
   {
      this.groundThickness = groundThickness;
   }

   public void setEstimatedNumberOfContactPoints(int estimatedNumberOfContactPoints)
   {
      this.estimatedNumberOfContactPoints = estimatedNumberOfContactPoints;
   }

   public void setGroundAngle(double groundAngle)
   {
      this.groundAngle = groundAngle;
   }

   public void setAddWalls(boolean addWalls)
   {
      this.addWalls = addWalls;
   }

   public void setCollisionGroup(int collisionGroup)
   {
      this.collisionGroup = collisionGroup;
   }

   public void setCollisionMask(int collisionMask)
   {
      this.collisionMask = collisionMask;
   }

   public void setFloorLength(double floorLength)
   {
      this.groundLength = floorLength;
   }

   public void setFloorWidth(double floorWidth)
   {
      this.groundWidth = floorWidth;
   }

   public void setFloorThickness(double floorThickness)
   {
      this.groundThickness = floorThickness;
   }

   public Robot createRobot()
   {
      Robot robot = new Robot("GroundRobot");
      RigidJoint baseJoint = new RigidJoint("base", new Vector3D(), robot);

      //    FloatingJoint baseJoint = new FloatingJoint("base", new Vector3d(), this);
      Link baseLink = new Link("base");
      baseLink.setMassAndRadiiOfGyration(100000000000.0, 100.0, 100.0, 100.0);

      Graphics3DObject baseLinkGraphics = new Graphics3DObject();
      baseLinkGraphics.translate(0.0, 0.0, -groundThickness);
      baseLinkGraphics.rotate(groundAngle, Axis.Y);
      baseLinkGraphics.addCube(groundLength, groundWidth, groundThickness, YoAppearance.Green());

      CollisionMeshDescription collisonMeshDescription = new CollisionMeshDescription();
      collisonMeshDescription.translate(0.0, 0.0, -groundThickness);
      collisonMeshDescription.rotate(groundAngle, Axis.Y);
      collisonMeshDescription.addCubeReferencedAtBottomMiddle(groundLength, groundWidth, groundThickness);
      collisonMeshDescription.setIsGround(true);
      collisonMeshDescription.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);

      collisonMeshDescription.setCollisionGroup(collisionGroup);
      collisonMeshDescription.setCollisionMask(collisionMask);

      baseLink.setLinkGraphics(baseLinkGraphics);
      baseLink.addCollisionMesh(collisonMeshDescription);

      baseJoint.setLink(baseLink);
      robot.addRootJoint(baseJoint);
      robot.addStaticLink(baseLink);

      return robot;
   }

}

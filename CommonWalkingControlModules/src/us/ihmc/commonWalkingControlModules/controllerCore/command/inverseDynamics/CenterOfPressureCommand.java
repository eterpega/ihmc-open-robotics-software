package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.screwTheory.RigidBody;

public class CenterOfPressureCommand implements InverseDynamicsCommand<CenterOfPressureCommand>
{
   private RigidBody contactingRigidBody;
   private String contactingRigidBodyName;
   private final Vector2D weightInSoleFrame = new Vector2D();
   private final Point2D desiredCoPInSoleFrame = new Point2D();

   private final FrameVector2d weightInWorldFrame = new FrameVector2d();
   private final FramePoint2d desiredCoPInWorldFrame = new FramePoint2d();

   public CenterOfPressureCommand()
   {
   }

   @Override
   public void set(CenterOfPressureCommand other)
   {
      this.weightInSoleFrame.set(other.weightInSoleFrame);
      this.desiredCoPInSoleFrame.set(other.desiredCoPInSoleFrame);

      this.contactingRigidBody = other.getContactingRigidBody();
      this.contactingRigidBodyName = other.contactingRigidBodyName;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.CENTER_OF_PRESSURE;
   }

   public void setContactingRigidBody(RigidBody contactingRigidBody)
   {
      this.contactingRigidBody = contactingRigidBody;
      this.contactingRigidBodyName = contactingRigidBody.getName();
   }

   public void setWeightInWorldFrame(FrameVector2d weightInWorldFrame)
   {
      this.weightInWorldFrame.set(weightInWorldFrame);
   }

   public void setDesiredCoPInWorldFrame(FramePoint2d desiredCoPInWorldFrame)
   {
      this.desiredCoPInWorldFrame.set(desiredCoPInWorldFrame);
   }

   public FrameVector2d getWeightInWorldFrame()
   {
      return weightInWorldFrame;
   }

   public FramePoint2d getDesiredCoPInWorldFrame()
   {
      return desiredCoPInWorldFrame;
   }

   public void setWeightInSoleFrame(Vector2D weightInSoleFrame)
   {
      this.weightInSoleFrame.set(weightInSoleFrame);
   }

   public void setDesiredCoPInSoleFrame(Point2D desiredCoPInSoleFrame)
   {
      this.desiredCoPInSoleFrame.set(desiredCoPInSoleFrame);
   }

   public Point2D getDesiredCoPInSoleFrame()
   {
      return desiredCoPInSoleFrame;
   }

   public Vector2D getWeightInSoleFrame()
   {
      return weightInSoleFrame;
   }

   public String getContactingRigidBodyName()
   {
      return contactingRigidBodyName;
   }

   public RigidBody getContactingRigidBody()
   {
      return contactingRigidBody;
   }

}

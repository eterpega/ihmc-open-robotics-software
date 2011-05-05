package us.ihmc.commonWalkingControlModules.controlModules.virtualSupportActuator;


import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DoubleSupportForceDistributor;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualSupportActuatorControlModule;
import us.ihmc.commonWalkingControlModules.kinematics.StanceFullLegJacobian;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Wrench;

public class SimpleVirtualSupportActuatorControlModule implements VirtualSupportActuatorControlModule
{
   private final DoubleSupportForceDistributor doubleSupportForceDistributor;
   private final SideDependentList<StanceFullLegJacobian> stanceFullLegJacobians;
   private final SideDependentList<ReferenceFrame> footFrames;
   private final SideDependentList<ReferenceFrame> footZUpFrames;

   public SimpleVirtualSupportActuatorControlModule(DoubleSupportForceDistributor doubleSupportForceDistributor, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<StanceFullLegJacobian> stanceFullLegJacobians)
   {
      this.doubleSupportForceDistributor = doubleSupportForceDistributor;
      this.stanceFullLegJacobians = new SideDependentList<StanceFullLegJacobian>(stanceFullLegJacobians);
      footFrames = new SideDependentList<ReferenceFrame>(referenceFrames.getFootFrame(RobotSide.LEFT),
            referenceFrames.getFootFrame(RobotSide.RIGHT));
      footZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
   }

   public void controlDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, SideDependentList<FramePoint2d> virtualToePoints,
         SideDependentList<Double> legStrengths, double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvis)
   {
      // distribute forces
      SideDependentList<Double> fZs = new SideDependentList<Double>();
      SideDependentList<FrameVector> torques = new SideDependentList<FrameVector>();
      doubleSupportForceDistributor.packForcesAndTorques(fZs, torques, fZOnPelvisInPelvisFrame, torqueOnPelvis, legStrengths);

      for (RobotSide robotSide : RobotSide.values())
      {
         LegTorques supportLegTorquesToPack = lowerBodyTorquesToPack.getLegTorques(robotSide);
         FramePoint2d vtp = new FramePoint2d(virtualToePoints.get(robotSide));
         double fZ = fZs.get(robotSide);
         FrameVector torque = torques.get(robotSide);

         controlSingleSupport(supportLegTorquesToPack, vtp, fZ, torque, null);
      }
   }


   public void controlSingleSupport(LegTorques supportLegTorquesToPack, FramePoint2d virtualToePoint, double fZOnPelvisInPelvisFrame,
                                    FrameVector torqueOnPelvisInPelvisFrame, Wrench upperBodyWrench)
   {
      RobotSide stanceSide = supportLegTorquesToPack.getRobotSide();

      StanceFullLegJacobian stanceFullLegJacobian = stanceFullLegJacobians.get(stanceSide);

      // compute Jacobian
      // TODO: Check all this reference frame stuff and make sure the Jacobians are using the correct frames. It may be that everything only works on flat ground.
      // TODO: Line below is BAD and we need a fix. VTPs need to be in foot frames, but we are computing them in ZUp Frames.
      // TODO: Still not OK. DoubleSupport uses Zup frames
      virtualToePoint.changeFrame(footZUpFrames.get(stanceSide));
      virtualToePoint = new FramePoint2d(footFrames.get(stanceSide), virtualToePoint.getX(), virtualToePoint.getY());
      stanceFullLegJacobian.computeJacobians(virtualToePoint);

      // compute a wrench in the nullspace of the VTP columns of the Jacobian
      Wrench wrenchOnPelvisInPelvisFrame = stanceFullLegJacobian.getWrenchInVTPNullSpace(fZOnPelvisInPelvisFrame, torqueOnPelvisInPelvisFrame);
      
      // compute joint torques
      stanceFullLegJacobian.packLegTorques(supportLegTorquesToPack, wrenchOnPelvisInPelvisFrame);
   }
}


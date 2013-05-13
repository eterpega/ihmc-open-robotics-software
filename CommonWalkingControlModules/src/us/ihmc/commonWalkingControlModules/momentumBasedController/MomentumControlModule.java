package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 4/25/13
 */
public interface MomentumControlModule
{
   void initialize();

   void reset();

   void compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportSide, Map<RigidBody, ? extends CylindricalContactState> cylinderContactStates);

   void resetGroundReactionWrenchFilter();

   void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration);

   void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData);

   void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData);

   SpatialForceVector getDesiredCentroidalMomentumRate();

   Map<RigidBody, Wrench> getExternalWrenches();

   void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench);
}

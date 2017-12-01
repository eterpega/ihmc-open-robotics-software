package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * The centroidal momentum can be written as \dot{h} = A * \dot{v} + \dot{A} * v. This class
 * calculates both the A matrix and the \dot{A} * v term. This implementation is going for
 * speed/efficiency, so the readability is not the best. For a more readable implementation, see
 * CentroidalMomentumMatrix for the A matrix and CentroidalMomentumRateADotVTerm for the \dot{A} * v
 * term.
 */

public class CentroidalMomentumRateTermCalculator
{
   private final InverseDynamicsJoint[] jointList;
   private final ReferenceFrame centerOfMassFrame;
   private final DenseMatrix64F centroidalMomentumMatrix;

   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final SpatialAccelerationVector tempSpatialAcceleration;

   private final Momentum[] unitMomenta;
   private final Momentum[] bodyMomenta;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RigidBody[] rigidBodies;
   private final RigidBody rootBody;
   private final DenseMatrix64F[] denseAdjTimesI;
   private final Momentum leftSide = new Momentum();
   private final Twist tempTwist = new Twist();
   private final Twist comTwist;
   private final Vector3D tempVector;
   private final Twist[] bodyTwists;
   private final DenseMatrix64F tempSpatialMotionMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F tempSpatialForceMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F tempInertiaMatrix = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F tempAdjoint = new DenseMatrix64F(6, 6);
   private final RotationMatrix tempMatrix3d = new RotationMatrix();
   private final Matrix3D tempPTilde = new Matrix3D();
   private final DenseMatrix64F v;
   private final DenseMatrix64F aDotV;
   private final double robotMass;
   private final boolean[][] isAncestorMapping;

   public CentroidalMomentumRateTermCalculator(RigidBody rootBody, ReferenceFrame centerOfMassFrame, DenseMatrix64F v, double robotMass)
   {
      this(rootBody, centerOfMassFrame, v, robotMass, null);
   }

   public CentroidalMomentumRateTermCalculator(RigidBody rootBody, ReferenceFrame centerOfMassFrame, DenseMatrix64F v, double robotMass, YoVariableRegistry registry)
   {
      this.rootBody = rootBody;
      this.jointList = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      this.centerOfMassFrame = centerOfMassFrame;
      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointList);
      this.v = v;
      this.centroidalMomentumMatrix = new DenseMatrix64F(6, nDegreesOfFreedom);
      this.unitMomenta = new Momentum[nDegreesOfFreedom];

      for (int i = 0; i < nDegreesOfFreedom; i++)
      {
         unitMomenta[i] = new Momentum(centerOfMassFrame);
      }

      this.tempSpatialAcceleration = new SpatialAccelerationVector();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame());
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, rootAcceleration, true, false, false);

      this.bodyMomenta = new Momentum[jointList.length];
      this.bodyTwists = new Twist[jointList.length];
      this.rigidBodies = new RigidBody[jointList.length];
      this.denseAdjTimesI = new DenseMatrix64F[jointList.length];

      isAncestorMapping = new boolean[jointList.length][jointList.length];

      for (int j = 0; j < jointList.length; j++)
      {
         bodyMomenta[j] = new Momentum(centerOfMassFrame);
         bodyTwists[j] = new Twist();
         rigidBodies[j] = jointList[j].getSuccessor();
         denseAdjTimesI[j] = new DenseMatrix64F(6, 6);

         RigidBody columnRigidBody = jointList[j].getSuccessor();
         for (int i = 0; i < jointList.length; i++)
         {
            RigidBody rowRigidBody = jointList[i].getSuccessor();
            isAncestorMapping[i][j] = ScrewTools.isAncestor(rowRigidBody, columnRigidBody);
         }
      }

      this.comTwist = new Twist(centerOfMassFrame, rootBody.getBodyFixedFrame(), centerOfMassFrame);
      comTwist.setToZero();
      this.tempVector = new Vector3D();
      this.aDotV = new DenseMatrix64F(6, 1);
      this.robotMass = robotMass;
      if (robotMass == 0)
      {
         throw new RuntimeException("Your robot has zero mass");
      }
      this.tempMatrix3d.setToZero();
   }

   public void compute()
   {
      spatialAccelerationCalculator.compute();

      precomputeAdjointTimesInertia();

      tempVector.setToZero();
      for (Momentum momentum : unitMomenta)
         momentum.setToZero();
      aDotV.zero();

      int column = 0;
      for (int j = 0; j < jointList.length; j++)
      {
         for (int k = 0; k < jointList[j].getDegreesOfFreedom(); k++)
         {

            for (int i = 0; i < jointList.length; i++)
            {
               if (isAncestorMapping[i][j])
               {
                  jointList[j].getUnitTwist(k, tempTwist);
                  tempTwist.changeFrame(rigidBodies[i].getInertia().getExpressedInFrame());

                  tempTwist.getMatrix(tempSpatialMotionMatrix, 0);
                  CommonOps.mult(denseAdjTimesI[i], tempSpatialMotionMatrix, tempMatrix);
                  unitMomenta[column].add(tempMatrix);
               }
            }
            // pack column into the centroidal momentum matrix
            unitMomenta[column].getMatrix(tempMatrix);
            MatrixTools.setMatrixBlock(centroidalMomentumMatrix, 0, column, tempMatrix, 0, 0, 6, 1, 1.0);

            // Multiply column by joint velocity and sum to compute center of mass velocity
            tempVector.scaleAdd(v.get(column), unitMomenta[column].getLinearPart(), tempVector);

            column++;
         }

         // Pack twist of body j w.r.t. elevator expressed in body j com
         rigidBodies[j].getBodyFixedFrame().getTwistRelativeToOther(rootBody.getBodyFixedFrame(), bodyTwists[j]);
         // Change expressed in frame to center of mass frame
         bodyTwists[j].changeFrame(centerOfMassFrame);

         // Ad^{T} * I * J * v
         tempTwist.set(bodyTwists[j]);
         tempTwist.changeFrame(jointList[j].getSuccessor().getInertia().getExpressedInFrame());
         tempTwist.getMatrix(tempSpatialMotionMatrix, 0);
         CommonOps.mult(denseAdjTimesI[j], tempSpatialMotionMatrix, tempMatrix);
         bodyMomenta[j].set(tempMatrix);

         computeADotVRightSide(j);
      }

      tempVector.scale(1.0 / robotMass);

      comTwist.setLinearPart(tempVector);

      computeADotVLeftSide();
   }

   private void computeADotVLeftSide()
   {
      for (int i = 0; i < bodyTwists.length; i++)
      {
         tempTwist.set(comTwist);
         tempTwist.sub(bodyTwists[i]);

         // Left multiply by ad^{T} : this is separated out rather than creating matrix objects to save computation time.
         // See method below for creating adjoint from twist for reference.
         // ad^{T} * Ad^{T} * I * J * v
         // The order of these cross products are backwards because I need -a x b and cross products are anticommutative,
         // so -a x b = b x a.
         leftSide.setToZero();
         tempVector.cross(bodyMomenta[i].getLinearPart(), tempTwist.getLinearPart());
         leftSide.addAngularPart(tempVector);
         tempVector.cross(bodyMomenta[i].getAngularPart(), tempTwist.getAngularPart());
         leftSide.addAngularPart(tempVector);
         tempVector.cross(bodyMomenta[i].getLinearPart(), tempTwist.getAngularPart());
         leftSide.addLinearPart(tempVector);
         //
         leftSide.getMatrix(tempSpatialForceMatrix);
         CommonOps.addEquals(aDotV, tempSpatialForceMatrix);
      }
   }

   private void computeADotVRightSide(int jointIndex)
   {
      // Here we calculate Jdot * v by computing the spatial acceleration with vddot = 0
      spatialAccelerationCalculator.getAccelerationOfBody(rigidBodies[jointIndex], tempSpatialAcceleration);
      tempSpatialAcceleration.getMatrix(tempSpatialMotionMatrix, 0);
      CommonOps.multAdd(denseAdjTimesI[jointIndex], tempSpatialMotionMatrix, aDotV);
   }

   /**
    * Precomputes Ad^{T} * I since it is used twice in Adot*v and once in computing A.
    */
   private void precomputeAdjointTimesInertia()
   {
      tempAdjoint.zero();
      for (int i = 0; i < jointList.length; i++)
      {
         rigidBodies[i].getInertia().getExpressedInFrame().getTransformToDesiredFrame(tempTransform, centerOfMassFrame);
         tempTransform.get(tempMatrix3d, tempVector);

         tempMatrix3d.get(0, 0, tempAdjoint);
         tempMatrix3d.get(3, 3, tempAdjoint);

         tempPTilde.setToTildeForm(tempVector); // aka, a skew-symmetric matrix
         tempPTilde.multiply(tempMatrix3d);
         tempPTilde.get(0, 3, tempAdjoint);

         rigidBodies[i].getInertia().getMatrix(tempInertiaMatrix);
         CommonOps.mult(tempAdjoint, tempInertiaMatrix, denseAdjTimesI[i]);
      }
   }

   public DenseMatrix64F getCentroidalMomentumMatrix()
   {
      return centroidalMomentumMatrix;
   }

   public DenseMatrix64F getADotVTerm()
   {
      return aDotV;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return centerOfMassFrame;
   }
}

package us.ihmc.valkyrie.kinematics.transmissions;

//~--- non-JDK imports --------------------------------------------------------

import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;
import us.ihmc.valkyrie.kinematics.util.ClosedFormJacobian;
import us.ihmc.valkyrie.roboNet.TurboDriver;

public class EfficientPushRodTransmission implements PushRodTransmissionInterface {
    private static final double INFINITY_THRESHOLD = 1e10;

    // Temporary matrix
    private double[][]               jacobian                  = new double[2][2];
    private final double[][]         jacobianTranspose         = new double[2][2];
    private final double[][]         jacobianInverse           = new double[2][2];
    private final double[][]         jacobianInvertedTranspose = new double[2][2];
    private final ClosedFormJacobian efficientPushrodTransmissionJacobian;
    private final double             reflect;

    public EfficientPushRodTransmission(PushRodTransmissionJoint pushRodTransmissionJoint, double reflect) {
        if (Math.abs(Math.abs(reflect) - 1.0) > 1e-7) {
            throw new RuntimeException("reflect must be 1.0 or -1.0");
        }

        this.reflect                         = reflect;
        efficientPushrodTransmissionJacobian = new ClosedFormJacobian(pushRodTransmissionJoint);
        efficientPushrodTransmissionJacobian.useFuteks(true);
    }

    private int numActuators() {
        return 2;
    }

    private int numJoints() {
        return 2;
    }

    private void assertTrue(boolean test) {
        if (!test) {
            throw new RuntimeException();
        }
    }

    private void checkInfinity(double value) {
        if (Math.abs(value) > INFINITY_THRESHOLD) {
            throw new RuntimeException("checkInfinity: Infinity value detected in supplied data structure!");
        }
    }

    private boolean invertMatrix(double[][] matrix, double[][] inverseToPack) {
        double det = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

        if (det != 0.0) {
            inverseToPack[0][0] = (1.0 / det) * matrix[1][1];
            inverseToPack[0][1] = (1.0 / det) * (-matrix[0][1]);
            inverseToPack[1][0] = (1.0 / det) * (-matrix[1][0]);
            inverseToPack[1][1] = (1.0 / det) * matrix[0][0];

            return true;
        } else {
            return false;
        }
    }

    private void transposeMatrix(double[][] matrix, double[][] transposeToPack) {
        transposeToPack[0][0] = matrix[0][0];
        transposeToPack[0][1] = matrix[1][0];
        transposeToPack[1][0] = matrix[0][1];
        transposeToPack[1][1] = matrix[1][1];
    }

    @Override
    public void actuatorToJointEffort(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData) {
        assertTrue((numActuators() == actuatorData.length) && (numJoints() == jointData.length));

        double actuatorForce0 = actuatorData[0].getEffort();
        double actuatorForce1 = actuatorData[1].getEffort();
        double pitchAngle     = jointData[0].getPosition();
        double rollAngle      = reflect * jointData[1].getPosition();

        jacobian = efficientPushrodTransmissionJacobian.getUpdatedTransform(rollAngle, pitchAngle);
        transposeMatrix(jacobian, jacobianTranspose);

        // tau = (J^T) * F
        double pitchTorque  = jacobianTranspose[0][0] * actuatorForce0 + jacobianTranspose[0][1] * actuatorForce1;
        double rollTorque = jacobianTranspose[1][0] * actuatorForce0 + jacobianTranspose[1][1] * actuatorForce1;

        jointData[0].setEffort(pitchTorque);
        jointData[1].setEffort(reflect * rollTorque);
    }

    @Override
    public void actuatorToJointVelocity(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data) {

        // Validation Velocity is used to validate the correct order and sign of the jacobian matrix elements.
        assertTrue((numActuators() == act_data.length) && (numJoints() == jnt_data.length));

        double actuatorVelocity0 = act_data[0].getVelocity();
        double actuatorVelocity1 = act_data[1].getVelocity();
        double pitchAngle        = jnt_data[0].getPosition();
        double rollAngle         = reflect * jnt_data[1].getPosition();

        jacobian = efficientPushrodTransmissionJacobian.getUpdatedTransform(rollAngle, pitchAngle);
        invertMatrix(jacobian, jacobianInverse);

        // theta_dot = J^-1 * x_dot
        double pitchVelocity  = jacobianInverse[0][0] * actuatorVelocity0 + jacobianInverse[0][1] * actuatorVelocity1;
        double rollVelocity = jacobianInverse[1][0] * actuatorVelocity0 + jacobianInverse[1][1] * actuatorVelocity1;

        jnt_data[0].setValidationVelocity(pitchVelocity);
        jnt_data[1].setValidationVelocity(reflect * rollVelocity);
    }

    @Override
    public void actuatorToJointPosition(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data) {

        // TODO Auto-generated method stub
    }

    @Override
    public void jointToActuatorEffort(TurboDriver[] actuatorData, ValkyrieJointInterface[] jointData) {
        assertTrue((numActuators() == actuatorData.length) && (numJoints() == jointData.length));

        double pitchAngle  = jointData[0].getPosition();
        double rollAngle   = reflect * jointData[1].getPosition();
        double pitchTorque = jointData[0].getDesiredEffort();
        double rollTorque  = reflect * jointData[1].getDesiredEffort();

        if ((Math.abs(pitchAngle) > INFINITY_THRESHOLD) || (Math.abs(rollAngle) > INFINITY_THRESHOLD)) {
            throw new RuntimeException("jointToActuatorEffort: pitchAngle or rollAngle is infinity!!\n");
        }

        jacobian = efficientPushrodTransmissionJacobian.getUpdatedTransform(rollAngle, pitchAngle);
        transposeMatrix(jacobian, jacobianTranspose);
        invertMatrix(jacobianTranspose, jacobianInvertedTranspose);

        // F = (J^T)^-1 * tau
        double actuatorForce0 = jacobianInvertedTranspose[0][0] * pitchTorque + jacobianInvertedTranspose[0][1] * rollTorque;
        double actuatorForce1 = jacobianInvertedTranspose[1][0] * pitchTorque + jacobianInvertedTranspose[1][1] * rollTorque;

        checkInfinity(actuatorForce0);
        checkInfinity(actuatorForce1);
        
        actuatorData[0].setEffortCommand(actuatorForce0);
        actuatorData[1].setEffortCommand(actuatorForce1);
    }

    @Override
    public void jointToActuatorVelocity(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data) {

        // TODO Auto-generated method stub
    }

    @Override
    public void jointToActuatorPosition(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data) {

        // TODO Auto-generated method stub
    }

    @Override
    public void updateMotorCurrents(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data) {
        assertTrue((numActuators() == act_data.length) && (numJoints() == jnt_data.length));
        jnt_data[0].setMotorCurrent(act_data[0].getCurrentIq());
        jnt_data[0].setCommandedMotorCurrent(act_data[0].getCurrentIqCmd());
        jnt_data[1].setMotorCurrent(act_data[1].getCurrentIq());
        jnt_data[1].setCommandedMotorCurrent(act_data[1].getCurrentIqCmd());
    }
}

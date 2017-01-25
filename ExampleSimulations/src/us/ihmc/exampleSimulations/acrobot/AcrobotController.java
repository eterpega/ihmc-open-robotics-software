package us.ihmc.exampleSimulations.acrobot;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;
import us.ihmc.tools.io.printing.PrintTools;

public class AcrobotController extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final AcrobotRobot robot;

   private final double m1;
   private final double I1;
   private final double l1;
   private final double l_1_sq;
   private final double lc1;
   private final double lc_1_sq;

   private final double m2;
   private final double I2;
   private final double l2;
   private final double l_2_sq;
   private final double lc2;
   private final double lc_2_sq;

   private final double g;

   private final YoFramePoint footViz = new YoFramePoint("FootLocation", worldFrame, registry);
   private final YoFrameVector computedReactionViz = new YoFrameVector("GroundReactionComputed", worldFrame, registry);

   private final DoubleYoVariable ankleAcceleration = new DoubleYoVariable("AnkleAcceleration", registry);
   private final DoubleYoVariable torque = new DoubleYoVariable("Torque", registry);

   public AcrobotController(AcrobotRobot robot, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.robot = robot;

      m1 = AcrobotRobot.LEG_MASS;
      I1 = AcrobotRobot.LEG_INERTIA;
      l1 = AcrobotRobot.LEG_LENGTH;
      l_1_sq = l1 * l1;
      lc1 = l1 / 2.0;
      lc_1_sq = lc1 * lc1;
      m2 = AcrobotRobot.BODY_MASS;
      I2 = AcrobotRobot.BODY_INERTIA;
      l2 = AcrobotRobot.BODY_LENGTH;
      l_2_sq = l2 * l2;
      lc2 = l2 / 2.0;
      lc_2_sq = lc2 * lc2;
      g = robot.getGravityZ();

      YoGraphicVector computedGRFYoGraphic = new YoGraphicVector("computedGRF", footViz, computedReactionViz, 0.025, YoAppearance.Red(), true);
      graphicsListRegistry.registerYoGraphic("Computed GRF", computedGRFYoGraphic);

      robot.setTheta1(Math.PI/2.0);
   }

   @Override
   public void doControl()
   {
      double q_1 = robot.getTheta1();
      double q_2 = robot.getTheta2();
      double q_1d = robot.getTheta1d();
      double q_2d = robot.getTheta2d();
      double q_1dd = robot.getTheta1dd();
      double q_2dd = robot.getTheta2dd();

      double s1 = Math.sin(q_1);
      double c1 = Math.cos(q_1);
      double s2 = Math.sin(q_2);
      double c2 = Math.cos(q_2);
      double s12 = Math.sin(q_1 + q_2);
      double c12 = Math.cos(q_1 + q_2);
      double q_1d_sq = q_1d * q_1d;
      double q_2d_sq = q_2d * q_2d;

      // use these forces to keep the robot position (inverted joint axis but working for some reason...)
      double xGRF = lc1 * m1 * (c1 * q_1dd - q_1d_sq * s1)
            + m2 * (c12 * lc2 * q_2dd - lc2 * q_2d * s12 * (q_1d + q_2d) - q_1d * (l1 * q_1d * s1 + lc2 * s12 * (q_1d + q_2d)) + q_1dd * (c1 * l1 + c12 * lc2));
      double zGRF = -g * (m1 + m2) - lc1 * m1 * (c1 * q_1d_sq + q_1dd * s1)
            - m2 * (c12 * lc2 * q_2d * (q_1d + q_2d) + lc2 * q_2dd * s12 + q_1d * (c1 * l1 * q_1d + c12 * lc2 * (q_1d + q_2d)) + q_1dd * (l1 * s1 + lc2 * s12));
      robot.setGRFx(xGRF);
      robot.setGRFz(zGRF);

      // validate the ankle acceleration
      double q1dd = -lc2 * m2 * (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2) * (g * s12 + l1 * q_1d * q_2d * s2 - l1 * q_1d_sq * s2)
            + (I2 + lc_2_sq * m2)
            * (g * l1 * m2 * s1 + g * lc1 * m1 * s1 + g * lc2 * m2 * s12 + 2.0 * l1 * lc2 * m2 * q_1d * q_2d * s2 + l1 * lc2 * m2 * q_2d_sq * s2)
            / ((I2 + lc_2_sq * m2) * (I1 + I2 + 2.0 * c2 * l1 * lc2 * m2 + l_1_sq * m2 + lc_1_sq * m1 + lc_2_sq * m2)
                  - (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2) * (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2));
      ankleAcceleration.set(q1dd);

      // validate torque
      double zero = -I2 * q_1dd - I2 * q_2dd - c2 * l1 * lc2 * m2 * q_1dd + g * lc2 * m2 * s12 + l1 * lc2 * m2 * q_1d * q_2d * s2 - l1 * lc2 * m2 * q_1d_sq * s2
            - lc_2_sq * m2 * q_1dd - lc_2_sq * m2 * q_2dd;
      torque.set(zero);

      footViz.set(robot.getContactPosition());
      computedReactionViz.set(robot.getContactForce());
   }

   public static void main(String[] args) throws UnreasonableAccelerationException
   {
      double m1 = AcrobotRobot.LEG_MASS;
      double I1 = AcrobotRobot.LEG_INERTIA;
      double l1 = AcrobotRobot.LEG_LENGTH;
      double m2 = AcrobotRobot.BODY_MASS;
      double I2 = AcrobotRobot.BODY_INERTIA;
      double l2 = AcrobotRobot.BODY_LENGTH;

      double q_1 = Math.PI / 2.0;
      double q_2 = 0.0;
      double q_1d = 0.0;
      double q_2d = 0.0;

      AcrobotRobot robot = new AcrobotRobot();
      robot.setTheta1(q_1);
      robot.setTheta1d(q_1d);
      robot.setTheta2(q_2);
      robot.setTheta2d(q_2d);
      double g = robot.getGravityZ();

      // compute helper variables
      double l_1_sq = l1 * l1;
      double lc1 = 0.5 * l1;
      double lc_1_sq = lc1 * lc1;
      double l_2_sq = l1 * l1;
      double lc2 = 0.5 * l2;
      double lc_2_sq = lc2 * lc2;
      double q_1d_sq = q_1d * q_1d;
      double q_2d_sq = q_2d * q_2d;
      double s1 = Math.sin(q_1);
      double c1 = Math.cos(q_1);
      double s2 = Math.sin(q_2);
      double c2 = Math.cos(q_2);
      double s12 = Math.sin(q_1 + q_2);
      double c12 = Math.cos(q_1 + q_2);

      // compute joint accelerations
      robot.doDynamicsButDoNotIntegrate();
      double expectedAnkleAcceleration = robot.getTheta1dd();
      double expectedHipAcceleration = robot.getTheta2dd();

      double ankleAcceleration = -lc2 * m2 * (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2) * (g * s12 + l1 * q_1d * q_2d * s2 - l1 * q_1d_sq * s2)
            + (I2 + lc_2_sq * m2)
                  * (g * l1 * m2 * s1 + g * lc1 * m1 * s1 + g * lc2 * m2 * s12 + 2.0 * l1 * lc2 * m2 * q_1d * q_2d * s2 + l1 * lc2 * m2 * q_2d_sq * s2)
                  / ((I2 + lc_2_sq * m2) * (I1 + I2 + 2.0 * c2 * l1 * lc2 * m2 + l_1_sq * m2 + lc_1_sq * m1 + lc_2_sq * m2)
                        - (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2) * (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2));
      double hipAcceleration = lc2 * m2 * (g * s12 + l1 * q_1d * q_2d * s2 - l1 * q_1d_sq * s2)
            * (I1 + I2 + 2.0 * c2 * l1 * lc2 * m2 + l_1_sq * m2 + lc_1_sq * m1 + lc_2_sq * m2)
            - (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2)
                  * (g * l1 * m2 * s1 + g * lc1 * m1 * s1 + g * lc2 * m2 * s12 + 2.0 * l1 * lc2 * m2 * q_1d * q_2d * s2 + l1 * lc2 * m2 * q_2d_sq * s2)
                  / ((I2 + lc_2_sq * m2) * (I1 + I2 + 2.0 * c2 * l1 * lc2 * m2 + l_1_sq * m2 + lc_1_sq * m1 + lc_2_sq * m2)
                        - (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2) * (I2 + c2 * l1 * lc2 * m2 + lc_2_sq * m2));

      PrintTools.info("Computed ankle acceleration: " + ankleAcceleration);
      PrintTools.info("Expected ankle acceleration: " + expectedAnkleAcceleration);
      PrintTools.info("Computed hip acceleration: " + hipAcceleration);
      PrintTools.info("Expected hip acceleration: " + expectedHipAcceleration);

   }

}

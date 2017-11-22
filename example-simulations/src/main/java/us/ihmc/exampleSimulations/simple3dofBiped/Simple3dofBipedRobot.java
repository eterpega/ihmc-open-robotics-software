package us.ihmc.exampleSimulations.simple3dofBiped;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoVariables.variable.YoDouble;

public class Simple3dofBipedRobot extends Robot
{
   private final double bodyLength = 1.0;
   private final double bodyRadius = 0.1;
   private final double bodyMass = 1.0;
   private final double bodyRogX = 0.1, bodyRogY = 0.1, bodyRogZ = 0.1;

   private final double hipOffsetY = 0.1;
   private final double thighLength = 0.5;
   private final double thighRadius = 0.06;
   private final double thighMass = 0.05;
   private final double thighRogX = thighLength / 3.0, thighRogY = thighLength / 3.0, thighRogZ = thighRadius;

   private final double shinLength = 0.6;
   private final double shinRadius = 0.03;
   private final double shinMass = 0.05;
   private final double shinRogX = shinLength / 3.0, shinRogY = shinLength / 3.0, shinRogZ = shinRadius;

   private final FloatingPlanarJoint bodyJoint;
   private final SideDependentList<PinJoint> hipJoints = new SideDependentList<>();
   private final SideDependentList<SliderJoint> kneeJoints = new SideDependentList<>();

   private final SideDependentList<GroundContactPoint> feetPoints = new SideDependentList<>();

   private final YoFramePoint capturePoint = new YoFramePoint("capturePoint", ReferenceFrame.getWorldFrame(), this.getRobotsYoVariableRegistry());
   private final SideDependentList<YoDouble> capturePointWithRespectToFeet = new SideDependentList<>();
   private final YoDouble capturePointWithRespectToBody = new YoDouble("capturePointBody", this.getRobotsYoVariableRegistry());

   public Simple3dofBipedRobot()
   {
      super("3dofBiped");

      bodyJoint = createBody();
      createLeg(RobotSide.LEFT, bodyJoint);
      createLeg(RobotSide.RIGHT, bodyJoint);

      setBodyPosition(0.0, 1.0);

      setHipJointPosition(RobotSide.LEFT, 0.2);
      setHipJointPosition(RobotSide.RIGHT, -0.2);

      setKneeJointPosition(RobotSide.LEFT, 1.0);
      setKneeJointPosition(RobotSide.RIGHT, 1.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         capturePointWithRespectToFeet.set(robotSide, new YoDouble("capturePoint" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Foot",
                                                                   this.getRobotsYoVariableRegistry()));
      }
   }

   private FloatingPlanarJoint createBody()
   {
      FloatingPlanarJoint bodyJoint = new FloatingPlanarJoint("bodyJoint", this);
      Link bodyLink = new Link("body");
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRogX, bodyRogY, bodyRogZ);
      Graphics3DObject bodyGraphics = new Graphics3DObject();
      bodyGraphics.translate(0.0, 0.0, bodyLength / 2.0 - bodyRadius);
      bodyGraphics.addCapsule(bodyRadius, bodyLength, YoAppearance.Gold());
      bodyLink.setLinkGraphics(bodyGraphics);
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);
      return bodyJoint;
   }

   private void setBodyPosition(double bodyXPosition, double bodyZPosition)
   {
      bodyJoint.setCartesianPosition(bodyXPosition, bodyZPosition);
   }

   private void createLeg(RobotSide robotSide, FloatingPlanarJoint planarJoint)
   {
      String sideName = robotSide.getCamelCaseNameForStartOfExpression();

      Vector3D hipOffset = new Vector3D(0.0, robotSide.negateIfRightSide(hipOffsetY), 0.0);
      PinJoint hipJoint = new PinJoint(sideName + "Hip", hipOffset, this, Axis.Y);
      Link thighLink = new Link(sideName + "Thigh");
      thighLink.setMassAndRadiiOfGyration(thighMass, thighRogX, thighRogY, thighRogZ);
      Graphics3DObject thighGraphics = new Graphics3DObject();
      thighGraphics.translate(0.0, 0.0, -thighLength / 2.0);
      thighGraphics.addCapsule(thighRadius, thighLength, YoAppearance.Red());
      thighLink.setLinkGraphics(thighGraphics);
      hipJoint.setLink(thighLink);
      planarJoint.addJoint(hipJoint);
      hipJoints.set(robotSide, hipJoint);

      SliderJoint kneeJoint = new SliderJoint(sideName + "Knee", new Vector3D(), this, new Vector3D(0.0, 0.0, -1.0));
      Link shinLink = new Link(sideName + "Shin");
      shinLink.setMassAndRadiiOfGyration(shinMass, shinRogX, shinRogY, shinRogZ);
      Graphics3DObject shinGraphics = new Graphics3DObject();
      shinGraphics.translate(0.0, 0.0, shinLength / 2.0);
      shinGraphics.addCapsule(shinRadius, shinLength, YoAppearance.Green());
      shinLink.setLinkGraphics(shinGraphics);
      kneeJoint.setLink(shinLink);
      hipJoint.addJoint(kneeJoint);
      kneeJoints.set(robotSide, kneeJoint);

      GroundContactPoint footPoint = new GroundContactPoint(sideName + "Foot", this.getRobotsYoVariableRegistry());
      kneeJoint.addGroundContactPoint(footPoint);
      feetPoints.set(robotSide, footPoint);
   }

   public double getBodyAngle()
   {
      return bodyJoint.getQ_rot().getDoubleValue();
   }

   public double getBodyAngularVelocity()
   {
      return bodyJoint.getQd_rot().getDoubleValue();
   }

   public void setHipJointPosition(RobotSide robotSide, double hipJointPosition)
   {
      hipJoints.get(robotSide).setQ(hipJointPosition);
   }

   public void setKneeJointPosition(RobotSide robotSide, double kneeJointPosition)
   {
      kneeJoints.get(robotSide).setQ(kneeJointPosition);
   }

   public void setHipTorque(RobotSide robotSide, double torque)
   {
      hipJoints.get(robotSide).setTau(torque);
   }

   public void setKneeForce(RobotSide robotSide, double force)
   {
      kneeJoints.get(robotSide).setTau(force);
   }

   public double getHipAngle(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQ();
   }

   public double getThighAngle(RobotSide robotSide)
   {
      return getBodyAngle() + getHipAngle(robotSide);
   }

   public double getThighAngularVelocity(RobotSide robotSide)
   {
      return getBodyAngularVelocity() + getHipAngularVelocity(robotSide);
   }

   public double getKneeLength(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQ();
   }

   public double getHipAngularVelocity(RobotSide robotSide)
   {
      return hipJoints.get(robotSide).getQD();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide).getQD();
   }

   public double getForwardVelocity()
   {
      return bodyJoint.getQd_t1().getDoubleValue();
   }

   public Point3D getFootPosition(RobotSide robotSide)
   {
      Point3D position = new Point3D();
      feetPoints.get(robotSide).getPosition(position);
      return position;
   }

   public void computeCapturePoint()
   {
      FrameVector3D capturePointVector = new FrameVector3D();
      bodyJoint.getVelocity(capturePointVector);

      capturePointVector.scale(0.3);
      capturePointWithRespectToBody.set(capturePointVector.getX());

      Vector3D translationToWorld = new Vector3D();
      bodyJoint.getTranslationToWorld(translationToWorld);
      capturePointVector.add(translationToWorld);
      capturePointVector.setZ(0.0);

      capturePoint.set(capturePointVector);

      for (RobotSide robotSide : RobotSide.values)
      {
         Point3D capturePointInFoot = capturePoint.getPoint3dCopy();
         capturePointInFoot.sub(this.getFootPosition(robotSide));

         capturePointWithRespectToFeet.get(robotSide).set(capturePointInFoot.getX());
      }
   }

   public double getCapturePointXWithRespectToFoot(RobotSide robotSide)
   {
      return capturePointWithRespectToFeet.get(robotSide).getDoubleValue();
   }

   public double getCapturePointXWithRespectToBody()
   {
      return capturePointWithRespectToBody.getDoubleValue();
   }

   public boolean hasFootMadeContact(RobotSide robotSide)
   {
      return feetPoints.get(robotSide).isInContact();
   }

}

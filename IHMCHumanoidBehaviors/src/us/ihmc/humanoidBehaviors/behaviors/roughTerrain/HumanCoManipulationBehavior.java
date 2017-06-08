package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import org.omg.CORBA.PRIVATE_MEMBER;

import jxl.read.biff.File;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.camera.CameraDataReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.tools.thread.ThreadTools;

public class HumanCoManipulationBehavior extends AbstractBehavior
{

   private FullHumanoidRobotModel fullHumanoidModel;
   private WalkingControllerParameters walkingControllerParameters;
   private HumanoidReferenceFrames referenceFrames;
   private DRCRobotSensorInformation robotSensorInfo;
   private ConcurrentListeningQueue<VideoPacket> cameraData = new ConcurrentListeningQueue<>(20);
   private SideDependentList<WristForceSensorFilteredUpdatable> wristSensorData;
   
   private BooleanYoVariable robotSideSelect = new BooleanYoVariable("SelectedRobotSide", registry);
   private RobotSide side;
   private DoubleYoVariable wristRawForceMagnitude = new DoubleYoVariable("WristForceMagnitude", registry);
   private YoFrameVector wristRawForceDirection = new YoFrameVector("WristForceDirection", HumanoidReferenceFrames.getWorldFrame(), registry);
   private DoubleYoVariable wristZeroForceMagnitude = new DoubleYoVariable("WristZeroForceMagnitude", registry);
   private YoFrameVector wristZeroForceDirection = new YoFrameVector("WristZeroForceDirection", HumanoidReferenceFrames.getWorldFrame(), registry);
   private DoubleYoVariable wristFilteredForceMagnitude = new DoubleYoVariable("WristFilteredForceMagnitude", registry);
   private YoFrameVector wristFilteredForceDirection = new YoFrameVector("WristFilteredForceDirection", HumanoidReferenceFrames.getWorldFrame(), registry);
   

   public HumanCoManipulationBehavior(CommunicationBridgeInterface communicationBridge, HumanoidReferenceFrames referenceFrames,
                                      FullHumanoidRobotModel fullHumanoidRobotModel, WalkingControllerParameters walkingControllerParameters,
                                      YoGraphicsListRegistry graphicsListRegistry, DRCRobotSensorInformation robotSensorInfo,
                                      SideDependentList<WristForceSensorFilteredUpdatable> wristSensors)
   {
      super(communicationBridge);
      this.fullHumanoidModel = fullHumanoidRobotModel;
      this.walkingControllerParameters = walkingControllerParameters;
      this.referenceFrames = referenceFrames;
      this.robotSensorInfo = robotSensorInfo;
      this.attachNetworkListeningQueue(cameraData, VideoPacket.class);
      this.wristSensorData = wristSensors;
      robotSideSelect.set(true);
   }

   @Override
   public void doControl()
   {
      updateRobotSide();
      readData();
      //PrintTools.info(wristFilteredForceMagnitude.toString() + " " + wristFilteredForceDirection.toString());

      /*
       * if(cameraData.isNewPacketAvailable()) { VideoPacket vidPack =
       * cameraData.getLatestPacket();
       * PrintTools.debug(vidPack.videoSource.toString()); if(!testImage) { try{
       * InputStream in = new ByteArrayInputStream(vidPack.getData());
       * ImageIO.write(ImageIO.read(in),"png", new java.io.File("testImage"));
       * }catch (IOException e) { } testImage = true; } }
       */
   }

   private void updateRobotSide()
   {
      if (robotSideSelect.getBooleanValue())
         side = RobotSide.LEFT;
      else
         side = RobotSide.RIGHT;
   }
   
   private void readRawData()
   {
      wristRawForceMagnitude.set(wristSensorData.get(side).getWristForceMagnitude().getDoubleValue());
      wristRawForceDirection.set(wristSensorData.get(side).getWristForceRawInWorld());
   }
   
   private void calibrateWristSensors()
   {
      readRawData();
      wristZeroForceMagnitude.set(wristRawForceMagnitude.getDoubleValue());
      wristZeroForceDirection.set(wristRawForceDirection.getFrameTuple());
   }
   
   private void readData()
   {
      readRawData();
      wristFilteredForceMagnitude.set(wristRawForceMagnitude.getDoubleValue() - wristZeroForceMagnitude.getDoubleValue());
      wristFilteredForceDirection.set(wristRawForceDirection.getFrameTuple());
      wristFilteredForceDirection.sub(wristZeroForceDirection.getFrameTuple());
   }
   
   @Override
   public void onBehaviorEntered()
   {
      calibrateWristSensors();
   }

   @Override
   public void onBehaviorAborted()
   {  
      // TODO Auto-generated method stub
   }

   @Override
   public void onBehaviorPaused()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorResumed()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void onBehaviorExited()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

}

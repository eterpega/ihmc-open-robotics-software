package us.ihmc.SdfLoader;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Camera;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU.IMUNoise;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU.IMUNoise.NoiseParameters;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Noise;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Range;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan.HorizontalScan;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan.VerticalScan;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.immutableRobotDescription.OneDoFJointDescription.LimitStops;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.immutableRobotDescription.*;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorLimitationParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorNoiseParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.xml.bind.JAXBException;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.*;

public class ImmutableRobotDescriptionFromSDFLoader
{
   private static final boolean SHOW_CONTACT_POINTS = true;
   private static final boolean SHOW_COM_REFERENCE_FRAMES = false;
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = false;
   private static final boolean SHOW_SENSOR_REFERENCE_FRAMES = false;

   public RobotDescription loadRobotDescriptionFromSDF(String modelName, InputStream inputStream, List<String> resourceDirectories, SDFDescriptionMutator mutator, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes,
         boolean enableTorqueVelocityLimits, boolean enableDamping)
   {
      GeneralizedSDFRobotModel generalizedSDFRobotModel = loadSDFFile(modelName, inputStream, resourceDirectories, mutator);
      return loadRobotDescriptionFromSDF(generalizedSDFRobotModel, sdfJointNameMap, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping);
   }

   public RobotDescription loadRobotDescriptionFromSDF(GeneralizedSDFRobotModel generalizedSDFRobotModel, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes, boolean enableTorqueVelocityLimits, boolean enableDamping)
   {
      List<String> resourceDirectories = generalizedSDFRobotModel.getResourceDirectories();

      String name = generalizedSDFRobotModel.getName();
      RobotDescriptionBuilder robotDescription = new RobotDescriptionBuilder().name(name);

      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();

      if (rootLinks.size() > 1)
      {
         throw new RuntimeException("Can only accomodate one root link for now");
      }

      SDFLinkHolder rootLink = rootLinks.get(0);

      Vector3d offset = new Vector3d();
      Quat4d orientation = new Quat4d();
      generalizedSDFRobotModel.getTransformToRoot().get(orientation, offset);
      FloatingJointDescriptionBuilder rootJointDescription = new FloatingJointDescriptionBuilder().name(rootLink.getName());

      LinkDescription rootLinkDescription = createLinkDescription(rootLink, new RigidBodyTransform(), useCollisionMeshes, resourceDirectories);
      rootJointDescription.link(rootLinkDescription);
      ConvertedSensors sensors = convertSensors(rootLink);
      rootJointDescription.addAllCameraSensors(sensors.cameras);
      rootJointDescription.addAllIMUSensors(sensors.imus);
      rootJointDescription.addAllLidarSensors(sensors.lidars);

      // Ground Contact Points:

      Map<String, Vector3d> sdfJointNameToGroundContactPoint = new HashMap<>();
      if (sdfJointNameMap != null)
      {
         for (ImmutablePair<String, Vector3d> jointContactPoint : sdfJointNameMap.getJointNameGroundContactPointMap())
            sdfJointNameToGroundContactPoint.put(jointContactPoint.getLeft(), jointContactPoint.getRight());
      }
      LinkedHashMap<String, Integer> groundContactPointCounters = new LinkedHashMap<>();

      if (sdfJointNameMap != null)
      {
         enableTorqueVelocityLimits = enableTorqueVelocityLimits && sdfJointNameMap.isTorqueVelocityLimitsEnabled();
      }

      for (SDFJointHolder child : rootLink.getChildren())
      {
         // System.out.println("Joint name: " + child.getName());

         Set<String> lastSimulatedJoints;

         if (sdfJointNameMap != null)
         {
            lastSimulatedJoints = sdfJointNameMap.getLastSimulatedJoints();
         }
         else
         {
            lastSimulatedJoints = new HashSet<>();
         }
         JointDescription convertedChildJoint = convertJointsRecursively(child, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping,
                                                                         lastSimulatedJoints, false, sdfJointNameToGroundContactPoint,
                                                                         groundContactPointCounters, sdfJointNameMap, resourceDirectories);
         rootJointDescription.addChildrenJoints(convertedChildJoint);
      }

      robotDescription.addChildrenJoints(rootJointDescription.build());

      return robotDescription.build();
   }

   // FIXME: graphics should be immutable as well
   private void convertLinkGraphics(JointDescription jointDescription) {
      if (SHOW_CONTACT_POINTS)
      {
         for (GroundContactPointDescription groundContactPointDescription : jointDescription.getGroundContactPoints())
         {
            Graphics3DObject graphics = jointDescription.getLink().getLinkGraphics();
            if (graphics == null) graphics = new Graphics3DObject();

            graphics.identity();
            graphics.translate(groundContactPointDescription.getOffsetFromJoint());
            double radius = 0.01;
            graphics.addSphere(radius, YoAppearance.Orange());
         }
      }

      List<SensorDescription> sensors = new ArrayList<>();
      sensors.addAll(jointDescription.getCameraSensors());
      sensors.addAll(jointDescription.getIMUSensors());
      sensors.addAll(jointDescription.getLidarSensors());
      for (SensorDescription sensor : sensors)
      {
         showCordinateSystem(jointDescription, sensor.getTransformToJoint());
      }
   }

   private JointContactPoints convertGroundContactPoints(String jointName, Map<String, Vector3d> sdfJointNameMap, LinkedHashMap<String, Integer> counters) {
      int count;
      if (counters.get(jointName) == null)
         count = 0;
      else
         count = counters.get(jointName);

      Vector3d gcOffset = sdfJointNameMap.get(jointName);
      if (gcOffset == null)
         return null;

      GroundContactPointDescription groundContactPoint = new GroundContactPointDescriptionBuilder()
            .name("gc_" + SDFConversionsHelper.sanitizeJointName(jointName) + "_" + count++)
            .offsetFromJoint(gcOffset)
            .build();
      ExternalForcePointDescription externalForcePoint = new GroundContactPointDescriptionBuilder()
            .name("ef_" + SDFConversionsHelper.sanitizeJointName(jointName) + "_" + count++)
            .offsetFromJoint(gcOffset)
            .build();


      counters.put(jointName, count);

      //            PrintTools.info("Joint Contact Point: " + jointContactPoint);

      return new JointContactPoints(groundContactPoint, externalForcePoint);
   }

   private static class JointContactPoints {
      final GroundContactPointDescription groundContactPointDescription;
      final ExternalForcePointDescription externalForcePointDescription;

      private JointContactPoints(GroundContactPointDescription groundContactPointDescription, ExternalForcePointDescription externalForcePointDescription)
      {
         this.groundContactPointDescription = groundContactPointDescription;
         this.externalForcePointDescription = externalForcePointDescription;
      }
   }

   private LinkDescription createLinkDescription(SDFLinkHolder link, RigidBodyTransform rotationTransform, boolean useCollisionMeshes, List<String> resourceDirectories)
   {
      LinkDescriptionBuilder scsLinkBuilder = new LinkDescriptionBuilder().name(link.getName());

      //TODO: Get collision meshes working.
      if (useCollisionMeshes)
      {
         LinkGraphicsDescription linkGraphicsDescription = new SDFGraphics3DObjectImmutable(link.getCollisions(), resourceDirectories, rotationTransform);

         scsLinkBuilder.linkGraphics(linkGraphicsDescription);
      }
      else if (link.getVisuals() != null)
      {
         LinkGraphicsDescription linkGraphicsDescription = new SDFGraphics3DObjectImmutable(link.getVisuals(), resourceDirectories, rotationTransform);
         scsLinkBuilder.linkGraphics(linkGraphicsDescription);
      }

      double mass = link.getMass();
      Matrix3d inertia = InertiaTools.rotate(rotationTransform, link.getInertia());
      Vector3d CoMOffset = new Vector3d(link.getCoMOffset());

      if (link.getJoint() != null)
      {
         if (isJointInNeedOfReducedGains(link.getJoint()))
         {
            inertia.mul(100.0);
         }
      }

      rotationTransform.transform(CoMOffset);

      scsLinkBuilder.centerOfMassOffset(CoMOffset);
      scsLinkBuilder.mass(mass);
      scsLinkBuilder.momentOfInertia(LinkDescription.convertMomentOfInertia(inertia));

      LinkDescription scsLink = scsLinkBuilder.build();
      if (SHOW_COM_REFERENCE_FRAMES)
      {
         scsLink.addCoordinateSystemToCOM(scsLink.getLinkGraphics(), 0.1);
      }
      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         scsLink.addEllipsoidFromMassProperties(scsLink.getLinkGraphics(), YoAppearance.Orange());
      }

      return scsLink;
   }

   private JointDescription convertJointsRecursively(SDFJointHolder joint, boolean useCollisionMeshes, boolean enableTorqueVelocityLimits,
                                                     boolean enableDamping, Set<String> lastSimulatedJoints, boolean doNotSimulateJoint,
                                                     Map<String, Vector3d> sdfJointNameToGroundContactPoint,
                                                     LinkedHashMap<String, Integer> groundContactPointCounters, SDFJointNameMap sdfJointNameMap,
                                                     List<String> resourceDirectories)
   {
      Vector3d jointAxis = new Vector3d(joint.getAxisInModelFrame());
      Vector3d offset = new Vector3d(joint.getOffsetFromParentJoint());

      RigidBodyTransform visualTransform = new RigidBodyTransform();
      visualTransform.setRotation(joint.getLinkRotation());

      String sanitizedJointName = SDFConversionsHelper.sanitizeJointName(joint.getName());

      JointDescription.Builder scsJointBuilder;


      switch (joint.getType())
      {
      case REVOLUTE:
         PinJointDescriptionBuilder pinJoint = new PinJointDescriptionBuilder().name(sanitizedJointName).offsetFromJoint(offset).axis(jointAxis);

         if (joint.hasLimits())
         {
            if (isJointInNeedOfReducedGains(joint))
            {
               pinJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 10.0, 2.5));
            }
            else
            {
               if ((joint.getContactKd() == 0.0) && (joint.getContactKp() == 0.0))
               {
                  pinJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 100.0, 20.0));
                  //                     pinJoint.setLimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 1000.0, 200.0);
               }
               else
               {
                  pinJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 0.0001 * joint.getContactKp(), 0.1 * joint.getContactKd()));
               }

               if (!Double.isNaN(joint.getVelocityLimit()))
                  pinJoint.velocityLimit(joint.getVelocityLimit()).velocityDamping(0.0);
               //System.out.println("SDFRobot: joint.getVelocityLimit()=" + joint.getVelocityLimit());

            }
         }

         if (enableDamping)
         {
            pinJoint.damping(joint.getDamping());
            pinJoint.stiction(joint.getFriction());
         }
         else
         {
            // TODO: Huh? What's this all about?
            //                  pinJoint.setDampingParameterOnly(joint.getDamping());
            //                  pinJoint.setStictionParameterOnly(joint.getFriction());
         }

         if (enableTorqueVelocityLimits)
         {
            if (!isJointInNeedOfReducedGains(joint))
            {
               if (!Double.isNaN(joint.getEffortLimit()))
               {
                  pinJoint.effortLimit(joint.getEffortLimit());
               }

               if (!Double.isNaN(joint.getVelocityLimit()))
               {
                  if (!isJointInNeedOfReducedGains(joint))
                  {
                     pinJoint.velocityLimit(joint.getVelocityLimit()).velocityDamping(500.0);
                  }
               }
            }
         }

         //               oneDoFJoints.put(joint.getName(), pinJoint);
         scsJointBuilder = pinJoint;

         break;

      case PRISMATIC:
         SliderJointDescriptionBuilder sliderJoint = new SliderJointDescriptionBuilder()
               .name(sanitizedJointName)
               .offsetFromJoint(offset)
               .axis(jointAxis);
         if (joint.hasLimits())
         {
            if ((joint.getContactKd() == 0.0) && (joint.getContactKp() == 0.0))
            {
               sliderJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 100.0, 20.0));
            }
            else
            {
               sliderJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), 0.0001 * joint.getContactKp(), joint.getContactKd()));
            }
         }

         if (enableDamping)
         {
            sliderJoint.damping(joint.getDamping());
            sliderJoint.stiction(joint.getFriction());
         }
         else
         {
            // TODO: Huh? What's this all about?
            //                  sliderJoint.setDampingParameterOnly(joint.getDamping());
         }

         scsJointBuilder = sliderJoint;
         break;

      default:
         throw new RuntimeException("Joint type not implemented: " + joint.getType());
      }

      if (doNotSimulateJoint) scsJointBuilder.dynamic(false);

      scsJointBuilder.link(createLinkDescription(joint.getChildLinkHolder(), visualTransform, useCollisionMeshes, resourceDirectories));

      ConvertedSensors convertedSensors = convertSensors(joint.getChildLinkHolder());
      scsJointBuilder.addAllCameraSensors(convertedSensors.cameras);
      scsJointBuilder.addAllIMUSensors(convertedSensors.imus);
      scsJointBuilder.addAllLidarSensors(convertedSensors.lidars);

      JointContactPoints groundContactPoints = convertGroundContactPoints(sanitizedJointName, sdfJointNameToGroundContactPoint, groundContactPointCounters);
      if (groundContactPoints != null)
      {
         scsJointBuilder.addGroundContactPoints(groundContactPoints.groundContactPointDescription);
         scsJointBuilder.addExternalForcePoints(groundContactPoints.externalForcePointDescription);
      }

      List<ForceSensorDescription> forceSensors = convertForceSensors(joint, sdfJointNameMap);
      scsJointBuilder.addAllForceSensors(forceSensors);

      if (!doNotSimulateJoint && lastSimulatedJoints.contains(joint.getName()))
      {
         doNotSimulateJoint = true;
      }

      for (SDFJointHolder child : joint.getChildLinkHolder().getChildren())
      {
         JointDescription childJoint = convertJointsRecursively(child, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping, lastSimulatedJoints,
                                                                doNotSimulateJoint, sdfJointNameToGroundContactPoint, groundContactPointCounters, sdfJointNameMap, resourceDirectories);
         scsJointBuilder.addChildrenJoints(childJoint);
      }
      JointDescription result = scsJointBuilder.build();
      convertLinkGraphics(result);
      return result;
   }

   ///TODO:
   ///XXX: pull these names from the sdfJointNameMap
   private boolean isJointInNeedOfReducedGains(SDFJointHolder pinJoint)
   {
      String jointName = pinJoint.getName();
      return jointName.contains("f0") || jointName.contains("f1") || jointName.contains("f2") || jointName.contains("f3") || jointName.contains("palm") || jointName.contains("finger");
   }

//   private void loadSDFFile()
//   {
//      resourceDirectories = Arrays.asList(sdfParameters.getResourceDirectories());
//      loadSDFFile(sdfParameters.getSdfAsInputStream(), resourceDirectories, null);
//   }

   private static GeneralizedSDFRobotModel loadSDFFile(String modelName, InputStream inputStream, List<String> resourceDirectories, SDFDescriptionMutator mutator)
   {
      GeneralizedSDFRobotModel generalizedSDFRobotModel;
      try
      {
         JaxbSDFLoader loader = new JaxbSDFLoader(inputStream, resourceDirectories, mutator);
         generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel(modelName);
//         resourceDirectories = generalizedSDFRobotModel.getResourceDirectories();
      }
      catch (FileNotFoundException | JAXBException e)
      {
         throw new RuntimeException("Cannot load model", e);
      }

      return generalizedSDFRobotModel;
   }

   private void showCordinateSystem(JointDescription scsJoint, RigidBodyTransform offsetFromLink)
   {
      if (SHOW_SENSOR_REFERENCE_FRAMES)
      {
         Graphics3DObject linkGraphics = scsJoint.getLink().getLinkGraphics();
         linkGraphics.identity();
         linkGraphics.transform(offsetFromLink);
         linkGraphics.addCoordinateSystem(1.0);
         linkGraphics.identity();
      }
   }

   private ConvertedSensors convertSensors(SDFLinkHolder child)
   {
      ConvertedSensors result = new ConvertedSensors();
      if (child.getSensors() != null)
      {
         for (SDFSensor sensor : child.getSensors())
         {
            switch (sensor.getType())
            {
            case "camera":
            case "multicamera":
               result.cameras.addAll(convertCameraMounts(sensor, child));
               break;
            case "imu":
               result.imus.addAll(convertIMUMounts(sensor, child));
               break;
            case "gpu_ray":
            case "ray":
               result.lidars.addAll(convertLidarMounts(sensor, child));
               break;
            }
         }
      }
      return result;
   }

   private static class ConvertedSensors {
      final List<CameraSensorDescription> cameras = new ArrayList<>();
      final List<IMUSensorDescription> imus = new ArrayList<>();
      final List<LidarSensorDescription> lidars = new ArrayList<>();
   }

   private List<CameraSensorDescription> convertCameraMounts(SDFSensor sensor, SDFLinkHolder child)
   {
      // TODO: handle left and right sides of multicamera
      final List<Camera> cameras = sensor.getCamera();

      List<CameraSensorDescription> result = new ArrayList<>();

      if (cameras != null)
      {
         for (Camera camera : cameras)
         {
            // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
            RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
            linkRotation.setTranslation(0.0, 0.0, 0.0);
            RigidBodyTransform linkToSensor = SDFConversionsHelper.poseToTransform(sensor.getPose());
            RigidBodyTransform sensorToCamera = SDFConversionsHelper.poseToTransform(camera.getPose());
            RigidBodyTransform linkToCamera = new RigidBodyTransform();
            linkToCamera.multiply(linkRotation, linkToSensor);
            linkToCamera.multiply(sensorToCamera);

            double fieldOfView = Double.parseDouble(camera.getHorizontalFov());
            double clipNear = Double.parseDouble(camera.getClip().getNear());
            double clipFar = Double.parseDouble(camera.getClip().getFar());
            String cameraName = sensor.getName() + "_" + camera.getName();
            CameraSensorDescriptionBuilder mount = new CameraSensorDescriptionBuilder()
                  .name(cameraName)
                  .transformToJoint(linkToCamera)
                  .fieldOfView(fieldOfView)
                  .clipNear(clipNear)
                  .clipFar(clipFar);

            int imageHeight = Integer.parseInt(camera.getImage().getHeight());
            int imageWidth = Integer.parseInt(camera.getImage().getWidth());

            mount.imageHeight(imageHeight);
            mount.imageWidth(imageWidth);

            result.add(mount.build());

//            SDFCamera sdfCamera = new SDFCamera(Integer.parseInt(camera.getImage().getWidth()), Integer.parseInt(camera.getImage().getHeight()));
            //            this.cameras.put(cameraName, sdfCamera);
         }
      }
      else
      {
         System.err.println("JAXB loader: No camera section defined for camera sensor " + sensor.getName() + ", ignoring sensor.");
      }
      return result;
   }

   private List<IMUSensorDescription> convertIMUMounts(SDFSensor sdfSensor, SDFLinkHolder child)
   {
      // TODO: handle left and right sides of multicamera
      final IMU imu = sdfSensor.getImu();
      List<IMUSensorDescription> result = new ArrayList<>();

      if (imu != null)
      {
         // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
         RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
         linkRotation.setTranslation(0.0, 0.0, 0.0);
         RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
         linkToSensorInZUp.multiply(linkRotation, SDFConversionsHelper.poseToTransform(sdfSensor.getPose()));

         IMUSensorDescriptionBuilder imuMount = new IMUSensorDescriptionBuilder()
               .name(child.getName() + "_" + sdfSensor.getName())
               .transformToJoint(linkToSensorInZUp);

         IMUNoise noise = imu.getNoise();
         if (noise != null)
         {
            if ("gaussian".equals(noise.getType()))
            {
               NoiseParameters accelerationNoise = noise.getAccel();
               NoiseParameters angularVelocityNoise = noise.getRate();

               GaussianParameter outAccelNoise = GaussianParameter.fromMeanStd(Double.parseDouble(accelerationNoise.getMean()), Double.parseDouble(accelerationNoise.getStddev()));
               GaussianParameter outAccelBias = GaussianParameter.fromMeanStd(Double.parseDouble(accelerationNoise.getBias_mean()), Double.parseDouble(accelerationNoise.getBias_stddev()));

               GaussianParameter outAngularVelNoise = GaussianParameter.fromMeanStd(Double.parseDouble(angularVelocityNoise.getMean()), Double.parseDouble(angularVelocityNoise.getStddev()));
               GaussianParameter outAngularValBias = GaussianParameter.fromMeanStd(Double.parseDouble(angularVelocityNoise.getBias_mean()), Double.parseDouble(angularVelocityNoise.getBias_stddev()));

               imuMount.accelerationNoise(outAccelNoise)
                     .accelerationBias(outAccelBias)
                     .angularVelocityNoise(outAngularVelNoise)
                     .angularVelocityBias(outAngularValBias);
            }
            else
            {
               throw new RuntimeException("Unknown IMU noise model: " + noise.getType());
            }
         }

         result.add(imuMount.build());

      }
      else
      {
         System.err.println("JAXB loader: No imu section defined for imu sensor " + sdfSensor.getName() + ", ignoring sensor.");
      }
      return result;
   }

   private List<LidarSensorDescription> convertLidarMounts(SDFSensor sensor, SDFLinkHolder child)
   {
      List<LidarSensorDescription> result = new ArrayList<>();
      Ray sdfRay = sensor.getRay();
      if (sdfRay == null)
      {
         System.err.println("SDFRobot: lidar not present in ray type sensor " + sensor.getName() + ". Ignoring this sensor.");
      }
      else
      {
         Range sdfRange = sdfRay.getRange();
         Scan sdfScan = sdfRay.getScan();
         double sdfMaxRange = Double.parseDouble(sdfRange.getMax());
         double sdfMinRange = Double.parseDouble(sdfRange.getMin());
         HorizontalScan sdfHorizontalScan = sdfScan.getHorizontal();
         VerticalScan sdfVerticalScan = sdfScan.getVertical();
         double sdfMaxSweepAngle = Double.parseDouble(sdfHorizontalScan.getMaxAngle());
         double sdfMinSweepAngle = Double.parseDouble(sdfHorizontalScan.getMinAngle());
         double sdfMaxHeightAngle = sdfVerticalScan == null ? 0.0 : Double.parseDouble(sdfVerticalScan.getMaxAngle());
         double sdfMinHeightAngle = sdfVerticalScan == null ? 0.0 : Double.parseDouble(sdfVerticalScan.getMinAngle());

         // double sdfAngularResolution = Double.parseDouble(sdfHorizontalScan.getSillyAndProbablyNotUsefulResolution());
         int sdfSamples = (Integer.parseInt(sdfHorizontalScan.getSamples()) / 3) * 3;
         int sdfScanHeight = sdfVerticalScan == null ? 1 : Integer.parseInt(sdfVerticalScan.getSamples());
         double sdfRangeResolution = Double.parseDouble(sdfRay.getRange().getResolution());

         final boolean sdfAlwaysOn = true;

         double sdfGaussianStdDev = 0.0;
         double sdfGaussianMean = 0.0;
         int sdfUpdateRate = (int) (1000.0 / Double.parseDouble(sensor.getUpdateRate()));

         Noise sdfNoise = sdfRay.getNoise();
         if (sdfNoise != null)
         {
            if ("gaussian".equals(sdfNoise.getType()))
            {
               sdfGaussianStdDev = Double.parseDouble(sdfNoise.getStddev());
               sdfGaussianMean = Double.parseDouble(sdfNoise.getMean());
            }
            else
            {
               System.err.println("Unknown noise model: " + sdfNoise.getType());
            }
         }

         //         System.err.println("[SDFRobot]: FIXME: Setting LIDAR angle to 0.5 pi due to current GPULidar limitations");
         //         sdfMinAngle = -Math.PI/4;
         //         sdfMaxAngle = Math.PI/4;

         LidarScanParameters polarDefinition = new LidarScanParameters(sdfSamples, sdfScanHeight, (float) sdfMinSweepAngle, (float) sdfMaxSweepAngle, (float) sdfMinHeightAngle, (float) sdfMaxHeightAngle, 0.0f,
                                                                       (float) sdfMinRange, (float) sdfMaxRange, 0.0f, 0L);

         // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
         RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
         linkRotation.setTranslation(0.0, 0.0, 0.0);
         RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
         linkToSensorInZUp.multiply(linkRotation, SDFConversionsHelper.poseToTransform(sensor.getPose()));

         SimulatedLIDARSensorNoiseParameters noiseParameters = new SimulatedLIDARSensorNoiseParameters();
         noiseParameters.setGaussianNoiseStandardDeviation(sdfGaussianStdDev);
         noiseParameters.setGaussianNoiseMean(sdfGaussianMean);

         SimulatedLIDARSensorLimitationParameters limitationParameters = new SimulatedLIDARSensorLimitationParameters();
         limitationParameters.setMaxRange(sdfMaxRange);
         limitationParameters.setMinRange(sdfMinRange);
         limitationParameters.setQuantization(sdfRangeResolution);

         SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
         updateParameters.setAlwaysOn(sdfAlwaysOn);
         updateParameters.setUpdatePeriodInMillis(sdfUpdateRate);

         LidarSensorDescription lidarMount = new LidarSensorDescriptionBuilder()
               .name(sensor.getName())
               .transformToJoint(linkToSensorInZUp)
               .lidarScanParameters(polarDefinition)
               .build();
         //         scsJoint.addLidarSensor(lidarMount);
         result.add(lidarMount);
      }
      return result;
   }

   private List<ForceSensorDescription> convertForceSensors(SDFJointHolder joint, SDFJointNameMap jointNameMap)
   {
      List<ForceSensorDescription> result = new ArrayList<>();
      if (joint.getForceSensors().size() > 0)
      {
         String[] jointNamesBeforeFeet = jointNameMap.getJointNamesBeforeFeet();

         String jointName = joint.getName();

         boolean jointIsParentOfFoot = false;
         for (String aJointNamesBeforeFeet : jointNamesBeforeFeet)
         {
            if (jointName.equals(aJointNamesBeforeFeet))
            {
               jointIsParentOfFoot = true;
               break;
            }
         }

         for (SDFForceSensor forceSensor : joint.getForceSensors())
         {
            ForceSensorDescriptionBuilder forceSensorDescription = new ForceSensorDescriptionBuilder().name(forceSensor.getName()).transformToJoint(forceSensor.getTransform());
            forceSensorDescription.useGroundContactPoints(jointIsParentOfFoot);
            result.add(forceSensorDescription.build());
         }
      }
      return result;
   }
}

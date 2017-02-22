package us.ihmc.modelFileLoaders.urdfLoader;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.*;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.extensions.URDFGazebo;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.tools.io.printing.PrintTools;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamConstants;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;
import javax.xml.transform.stream.StreamSource;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RobotDescriptionFromURDFLoader
{
   private final JAXBContext coreContext;
   private final Unmarshaller coreUnmashaller;
   private String modelName;
   private List<String> resourceDirectories;

   public RobotDescriptionFromURDFLoader()
   {
      try
      {
         coreContext = JAXBContext.newInstance("us.ihmc.modelFileLoaders.urdfLoader.xmlDescription");
         coreUnmashaller = coreContext.createUnmarshaller();
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public RobotDescription loadRobotDescriptionFromURDF(String modelName, URL modelFileResourceURL, List<String> resourceDirectories, JointNameMap jointNameMap,
                                                        boolean useCollisionMeshes)
   {
      RobotDescription robotDescription;
      try
      {
         robotDescription = new RobotDescription(modelName);
         URDFRobot urdfRobot = (URDFRobot) coreUnmashaller.unmarshal(modelFileResourceURL);
         List<URDFGazebo> gazeboTags = getGazeboExtensions(modelFileResourceURL);

         HashMap<String, URDFJoint> allJointsFromURDFRobot = new HashMap<>();
         HashMap<String, URDFLink> allLinksFromURDFRobot = new HashMap<>();
         HashMap<String, URDFMaterialGlobal> allGlobalMaterialsFromURDFRobot = new HashMap<>();
         HashMap<String, URDFTransmission> allTransmissionFromURDFRobot = new HashMap<>();

         ModelFileLoaderConversionsHelper.getGlobalLinksJointsMaterialsAndTransmissionFromURDFRobot(urdfRobot, allJointsFromURDFRobot, allLinksFromURDFRobot,
                                                                                                    allGlobalMaterialsFromURDFRobot,
                                                                                                    allTransmissionFromURDFRobot);

         for (Map.Entry<String, URDFLink> stringURDFLinkEntry : allLinksFromURDFRobot.entrySet())
         {
            String linkName = stringURDFLinkEntry.getKey();
            URDFLink link = stringURDFLinkEntry.getValue();

            LinkDescription linkDescription = new LinkDescription(linkName);

            ArrayList<URDFVisual> linkVisuals = new ArrayList<>();
            ArrayList<URDFCollision> linkCollisions = new ArrayList<>();

            URDFInertial inertialInformationForURDFLink = ModelFileLoaderConversionsHelper
                  .getInertialInformationAndPackVisualsAndCollisionsForURDFLink(link, linkVisuals, linkCollisions);
            setupInertialInformationForLink(linkName, linkDescription, inertialInformationForURDFLink);

            for (URDFVisual linkVisual : linkVisuals)
            {
               URDFPose origin = linkVisual.getOrigin();

               Vector3D centerOfMassOffset;
               QuaternionReadOnly visualRotation;

               if (origin != null)
               {
                  Pair<Vector3D, QuaternionReadOnly> poseFromURDFPose = ModelFileLoaderConversionsHelper.getPoseFromURDFPose(linkName, origin);
                  centerOfMassOffset = poseFromURDFPose.getLeft();
                  visualRotation = poseFromURDFPose.getRight();
               }
               else
               {
                  centerOfMassOffset = new Vector3D();
                  visualRotation = new Quaternion();
               }

               URDFGeometry geometry = linkVisual.getGeometry();
               URDFMaterial material = linkVisual.getMaterial();
            }
         }

         for (Map.Entry<String, URDFJoint> stringURDFJointEntry : allJointsFromURDFRobot.entrySet())
         {
            System.out.println(stringURDFJointEntry.getKey());
         }

         for (Map.Entry<String, URDFMaterialGlobal> stringURDFMaterialGlobalEntry : allGlobalMaterialsFromURDFRobot.entrySet())
         {
            System.out.println(stringURDFMaterialGlobalEntry.getKey());
         }

         for (Map.Entry<String, URDFTransmission> stringURDFTransmissionEntry : allTransmissionFromURDFRobot.entrySet())
         {
            System.out.println(stringURDFTransmissionEntry.getKey());
         }
      }
      catch (Throwable e)
      {
         robotDescription = null;
         throw new RuntimeException("Could not load robot model " + modelName, e);
      }

      return robotDescription;
   }

   private void setupInertialInformationForLink(String linkName, LinkDescription linkDescription, URDFInertial inertialInformationForURDFLink)
   {
      if (inertialInformationForURDFLink != null)
      {
         URDFMass mass = inertialInformationForURDFLink.getMass();
         URDFInertia inertia = inertialInformationForURDFLink.getInertia();
         URDFPose origin = inertialInformationForURDFLink.getOrigin();

         Vector3D centerOfMassOffset;
         QuaternionReadOnly linkInertiaRotation;
         if (origin != null)
         {
            Pair<Vector3D, QuaternionReadOnly> poseFromURDFPose = ModelFileLoaderConversionsHelper.getPoseFromURDFPose(linkName, origin);
            centerOfMassOffset = poseFromURDFPose.getLeft();
            linkInertiaRotation = poseFromURDFPose.getRight();
         }
         else
         {
            centerOfMassOffset = new Vector3D();
            linkInertiaRotation = new Quaternion();
         }

         parseMassProperties(linkName, linkDescription, mass, centerOfMassOffset);

         parseInertiaProperties(linkName, linkDescription, inertia, linkInertiaRotation);
      }
      else
      {
         PrintTools.warn(this, "No <intertial> tag for link " + linkName + ", setting all inertial properties to 0.0");
         linkDescription.setMomentOfInertia(0.0, 0.0, 0.0);
         linkDescription.setMass(0.0);
         linkDescription.setCenterOfMassOffset(0.0, 0.0, 0.0);
      }
   }

   private void parseInertiaProperties(String linkName, LinkDescription linkDescription, URDFInertia inertia, QuaternionReadOnly rotation)
   {
      Matrix3D momentOfInertia = new Matrix3D();

      if (inertia != null)
      {
         momentOfInertia.set(inertia.getIxx(), inertia.getIxy(), inertia.getIxz(), inertia.getIxy(), inertia.getIyy(), inertia.getIyz(), inertia.getIxz(),
                             inertia.getIyz(), inertia.getIzz());

         rotation.transform(momentOfInertia);
         linkDescription.setMomentOfInertia(momentOfInertia);
      }
   }

   private void parseMassProperties(String linkName, LinkDescription linkDescription, URDFMass mass, Vector3D centerOfMassOffset)
   {
      if (mass != null)
      {
         linkDescription.setMass(mass.getValue());
      }

      linkDescription.setCenterOfMassOffset(centerOfMassOffset);
   }

   private List<URDFGazebo> getGazeboExtensions(URL modelFileResourceURL) throws XMLStreamException, IOException, JAXBException
   {
      XMLInputFactory xmlInputFactory = XMLInputFactory.newFactory();
      XMLStreamReader xmlStreamReader = xmlInputFactory.createXMLStreamReader(new StreamSource(modelFileResourceURL.openStream()));
      ArrayList<URDFGazebo> gazeboTags = new ArrayList<>();
      Unmarshaller unmarshaller = JAXBContext.newInstance(URDFGazebo.class).createUnmarshaller();

      int eventType = xmlStreamReader.next();
      while (xmlStreamReader.hasNext())
      {
         if (eventType == XMLStreamConstants.START_ELEMENT)
         {
            if (xmlStreamReader.getLocalName().equals("gazebo"))
            {
               JAXBElement<URDFGazebo> unmarshal = unmarshaller.unmarshal(xmlStreamReader, URDFGazebo.class);

               gazeboTags.add(unmarshal.getValue());
            }
            eventType = xmlStreamReader.next();
         }
         else
         {
            eventType = xmlStreamReader.next();
         }
      }
      return gazeboTags;
   }
}

package us.ihmc.modelFileLoaders.urdfLoader;

import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFJoint;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFLink;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFRobot;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.extensions.URDFGazebo;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;

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
import java.util.List;

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
      }
      catch (Throwable e)
      {
         robotDescription = null;
         throw new RuntimeException("Could not load robot model " + modelName, e);
      }

      return robotDescription;
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

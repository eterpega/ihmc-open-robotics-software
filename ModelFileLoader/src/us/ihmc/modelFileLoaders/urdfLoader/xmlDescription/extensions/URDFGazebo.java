package us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.extensions;

import org.w3c.dom.Element;

import javax.xml.bind.annotation.XmlAnyElement;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@XmlRootElement(name = "gazebo")
public class URDFGazebo
{
   private String reference;

   // Link extensions:
   private double dampingFactor = Double.NaN;
   private double maxVel = Double.NaN;
   private double minDepth = Double.NaN;
   private double mu1 = Double.NaN;
   private double mu2 = Double.NaN;
   private String fdir1 = null;
   private double kp = Double.NaN;
   private double kd = Double.NaN;
   private boolean selfCollide = false;
   private int maxContacts = Integer.MIN_VALUE;
   private double laserRetro = Double.NaN;

   // Joint extensions:
   private double stopCfm = Double.NaN;
   private double stopErp = Double.NaN;
   private boolean provideFeedback = false;
   private boolean implicitSpringDamper = false;
   private boolean cfmDamping = false;
   private double fudgeFactor = Double.NaN;

   // All other tags, e.g. all inline SDF elements
   private List<Element> extensions;

   @XmlAttribute(name = "reference")
   public void setReference(String reference)
   {
      this.reference = reference;
   }

   public String getReference()
   {
      return reference;
   }

   public double getDampingFactor()
   {
      return dampingFactor;
   }

   @XmlElement(name = "dampingFactor", namespace = "http://www.ros.org")
   public void setDampingFactor(double dampingFactor)
   {
      this.dampingFactor = dampingFactor;
   }

   public double getMaxVel()
   {
      return maxVel;
   }

   @XmlElement(name = "maxVel", namespace = "http://www.ros.org")
   public void setMaxVel(double maxVel)
   {
      this.maxVel = maxVel;
   }

   public double getMinDepth()
   {
      return minDepth;
   }

   @XmlElement(name = "minDepth", namespace = "http://www.ros.org")
   public void setMinDepth(double minDepth)
   {
      this.minDepth = minDepth;
   }

   public double getMu1()
   {
      return mu1;
   }

   @XmlElement(name = "mu1", namespace = "http://www.ros.org")
   public void setMu1(double mu1)
   {
      this.mu1 = mu1;
   }

   public double getMu2()
   {
      return mu2;
   }

   @XmlElement(name = "mu2", namespace = "http://www.ros.org")
   public void setMu2(double mu2)
   {
      this.mu2 = mu2;
   }

   public String getFdir1()
   {
      return fdir1;
   }

   @XmlElement(name = "fdir1", namespace = "http://www.ros.org")
   public void setFdir1(String fdir1)
   {
      this.fdir1 = fdir1;
   }

   public double getKp()
   {
      return kp;
   }

   @XmlElement(name = "kp", namespace = "http://www.ros.org")
   public void setKp(double kp)
   {
      this.kp = kp;
   }

   public double getKd()
   {
      return kd;
   }

   @XmlElement(name = "kd", namespace = "http://www.ros.org")
   public void setKd(double kd)
   {
      this.kd = kd;
   }

   public boolean isSelfCollide()
   {
      return selfCollide;
   }

   @XmlElement(name = "selfCollide", namespace = "http://www.ros.org")
   public void setSelfCollide(boolean selfCollide)
   {
      this.selfCollide = selfCollide;
   }

   public int getMaxContacts()
   {
      return maxContacts;
   }

   @XmlElement(name = "maxContacts", namespace = "http://www.ros.org")
   public void setMaxContacts(int maxContacts)
   {
      this.maxContacts = maxContacts;
   }

   public double getLaserRetro()
   {
      return laserRetro;
   }

   @XmlElement(name = "laserRetro", namespace = "http://www.ros.org")
   public void setLaserRetro(double laserRetro)
   {
      this.laserRetro = laserRetro;
   }

   public double getStopCfm()
   {
      return stopCfm;
   }

   @XmlElement(name = "stopCfm", namespace = "http://www.ros.org")
   public void setStopCfm(double stopCfm)
   {
      this.stopCfm = stopCfm;
   }

   public double getStopErp()
   {
      return stopErp;
   }

   @XmlElement(name = "stopErp", namespace = "http://www.ros.org")
   public void setStopErp(double stopErp)
   {
      this.stopErp = stopErp;
   }

   public boolean isProvideFeedback()
   {
      return provideFeedback;
   }

   @XmlElement(name = "provideFeedback", namespace = "http://www.ros.org")
   public void setProvideFeedback(boolean provideFeedback)
   {
      this.provideFeedback = provideFeedback;
   }

   public boolean isImplicitSpringDamper()
   {
      return implicitSpringDamper;
   }

   @XmlElement(name = "implicitSpringDamper", namespace = "http://www.ros.org")
   public void setImplicitSpringDamper(boolean implicitSpringDamper)
   {
      this.implicitSpringDamper = implicitSpringDamper;
   }

   public boolean isCfmDamping()
   {
      return cfmDamping;
   }

   @XmlElement(name = "cfmDamping", namespace = "http://www.ros.org")
   public void setCfmDamping(boolean cfmDamping)
   {
      this.cfmDamping = cfmDamping;
   }

   public double getFudgeFactor()
   {
      return fudgeFactor;
   }

   @XmlElement(name = "fudgeFactor", namespace = "http://www.ros.org")
   public void setFudgeFactor(double fudgeFactor)
   {
      this.fudgeFactor = fudgeFactor;
   }

   @XmlAnyElement
   public void setExtensions(List<Element> extensions)
   {
      this.extensions = extensions;
   }

   public List<Element> getExtensions()
   {
      return extensions;
   }
}

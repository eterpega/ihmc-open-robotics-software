
package us.ihmc.modelFileLoaders.urdfLoader.xmlDescription;

import java.util.ArrayList;
import java.util.List;
import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlElements;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for anonymous complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType>
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;sequence maxOccurs="unbounded" minOccurs="0">
 *         &lt;element name="joint" type="{http://www.ros.org}joint" maxOccurs="unbounded" minOccurs="0"/>
 *         &lt;element name="link" type="{http://www.ros.org}link" maxOccurs="unbounded" minOccurs="0"/>
 *         &lt;element name="material" type="{http://www.ros.org}material_global" maxOccurs="unbounded" minOccurs="0"/>
 *         &lt;element name="transmission" type="{http://www.ros.org}transmission" maxOccurs="unbounded" minOccurs="0"/>
 *       &lt;/sequence>
 *       &lt;attribute name="name" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="version" type="{http://www.w3.org/2001/XMLSchema}string" default="1.0" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "", propOrder = {
    "jointAndLinkAndMaterial"
})
@XmlRootElement(name = "robot", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class URDFRobot {

    @XmlElements({
        @XmlElement(name = "joint", namespace = "http://www.ros.org", type = URDFJoint.class),
        @XmlElement(name = "link", namespace = "http://www.ros.org", type = URDFLink.class),
        @XmlElement(name = "material", namespace = "http://www.ros.org", type = URDFMaterialGlobal.class),
        @XmlElement(name = "transmission", namespace = "http://www.ros.org", type = URDFTransmission.class)
    })
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected List<Object> jointAndLinkAndMaterial;
    @XmlAttribute(name = "name", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String name;
    @XmlAttribute(name = "version")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String version;

    /**
     * Gets the value of the jointAndLinkAndMaterial property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the jointAndLinkAndMaterial property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getJointAndLinkAndMaterial().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link URDFJoint }
     * {@link URDFLink }
     * {@link URDFMaterialGlobal }
     * {@link URDFTransmission }
     * 
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public List<Object> getJointAndLinkAndMaterial() {
        if (jointAndLinkAndMaterial == null) {
            jointAndLinkAndMaterial = new ArrayList<Object>();
        }
        return this.jointAndLinkAndMaterial;
    }

    /**
     * Gets the value of the name property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public String getName() {
        return name;
    }

    /**
     * Sets the value of the name property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setName(String value) {
        this.name = value;
    }

    /**
     * Gets the value of the version property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public String getVersion() {
        if (version == null) {
            return "1.0";
        } else {
            return version;
        }
    }

    /**
     * Sets the value of the version property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setVersion(String value) {
        this.version = value;
    }

}

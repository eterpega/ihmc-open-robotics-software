
package us.ihmc.modelFileLoaders.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for inertial complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="inertial">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;all>
 *         &lt;element name="origin" type="{http://www.ros.org}pose" minOccurs="0"/>
 *         &lt;element name="mass" type="{http://www.ros.org}mass" minOccurs="0"/>
 *         &lt;element name="inertia" type="{http://www.ros.org}inertia" minOccurs="0"/>
 *       &lt;/all>
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "inertial", namespace = "http://www.ros.org", propOrder = {

})
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class URDFInertial {

    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected URDFPose origin;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected URDFMass mass;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected URDFInertia inertia;

    /**
     * Gets the value of the origin property.
     * 
     * @return
     *     possible object is
     *     {@link URDFPose }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public URDFPose getOrigin() {
        return origin;
    }

    /**
     * Sets the value of the origin property.
     * 
     * @param value
     *     allowed object is
     *     {@link URDFPose }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setOrigin(URDFPose value) {
        this.origin = value;
    }

    /**
     * Gets the value of the mass property.
     * 
     * @return
     *     possible object is
     *     {@link URDFMass }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public URDFMass getMass() {
        return mass;
    }

    /**
     * Sets the value of the mass property.
     * 
     * @param value
     *     allowed object is
     *     {@link URDFMass }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMass(URDFMass value) {
        this.mass = value;
    }

    /**
     * Gets the value of the inertia property.
     * 
     * @return
     *     possible object is
     *     {@link URDFInertia }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public URDFInertia getInertia() {
        return inertia;
    }

    /**
     * Sets the value of the inertia property.
     * 
     * @param value
     *     allowed object is
     *     {@link URDFInertia }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setInertia(URDFInertia value) {
        this.inertia = value;
    }

}

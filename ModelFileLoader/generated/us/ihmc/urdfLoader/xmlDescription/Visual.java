
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for visual complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="visual">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;sequence>
 *         &lt;element name="origin" type="{http://www.ros.org}pose" minOccurs="0"/>
 *         &lt;element name="geometry" type="{http://www.ros.org}geometry"/>
 *         &lt;element name="material" type="{http://www.ros.org}material" minOccurs="0"/>
 *       &lt;/sequence>
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "visual", namespace = "http://www.ros.org", propOrder = {
    "origin",
    "geometry",
    "material"
})
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Visual {

    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Pose origin;
    @XmlElement(namespace = "http://www.ros.org", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Geometry geometry;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Material material;

    /**
     * Gets the value of the origin property.
     * 
     * @return
     *     possible object is
     *     {@link Pose }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Pose getOrigin() {
        return origin;
    }

    /**
     * Sets the value of the origin property.
     * 
     * @param value
     *     allowed object is
     *     {@link Pose }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setOrigin(Pose value) {
        this.origin = value;
    }

    /**
     * Gets the value of the geometry property.
     * 
     * @return
     *     possible object is
     *     {@link Geometry }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Geometry getGeometry() {
        return geometry;
    }

    /**
     * Sets the value of the geometry property.
     * 
     * @param value
     *     allowed object is
     *     {@link Geometry }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setGeometry(Geometry value) {
        this.geometry = value;
    }

    /**
     * Gets the value of the material property.
     * 
     * @return
     *     possible object is
     *     {@link Material }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Material getMaterial() {
        return material;
    }

    /**
     * Sets the value of the material property.
     * 
     * @param value
     *     allowed object is
     *     {@link Material }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMaterial(Material value) {
        this.material = value;
    }

}


package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for geometry complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="geometry">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;choice>
 *         &lt;element name="box" type="{http://www.ros.org}box"/>
 *         &lt;element name="cylinder" type="{http://www.ros.org}cylinder"/>
 *         &lt;element name="sphere" type="{http://www.ros.org}sphere"/>
 *         &lt;element name="mesh" type="{http://www.ros.org}mesh"/>
 *       &lt;/choice>
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "geometry", namespace = "http://www.ros.org", propOrder = {
    "box",
    "cylinder",
    "sphere",
    "mesh"
})
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Geometry {

    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Box box;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Cylinder cylinder;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Sphere sphere;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Mesh mesh;

    /**
     * Gets the value of the box property.
     * 
     * @return
     *     possible object is
     *     {@link Box }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Box getBox() {
        return box;
    }

    /**
     * Sets the value of the box property.
     * 
     * @param value
     *     allowed object is
     *     {@link Box }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setBox(Box value) {
        this.box = value;
    }

    /**
     * Gets the value of the cylinder property.
     * 
     * @return
     *     possible object is
     *     {@link Cylinder }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Cylinder getCylinder() {
        return cylinder;
    }

    /**
     * Sets the value of the cylinder property.
     * 
     * @param value
     *     allowed object is
     *     {@link Cylinder }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setCylinder(Cylinder value) {
        this.cylinder = value;
    }

    /**
     * Gets the value of the sphere property.
     * 
     * @return
     *     possible object is
     *     {@link Sphere }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Sphere getSphere() {
        return sphere;
    }

    /**
     * Sets the value of the sphere property.
     * 
     * @param value
     *     allowed object is
     *     {@link Sphere }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setSphere(Sphere value) {
        this.sphere = value;
    }

    /**
     * Gets the value of the mesh property.
     * 
     * @return
     *     possible object is
     *     {@link Mesh }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Mesh getMesh() {
        return mesh;
    }

    /**
     * Sets the value of the mesh property.
     * 
     * @param value
     *     allowed object is
     *     {@link Mesh }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMesh(Mesh value) {
        this.mesh = value;
    }

}

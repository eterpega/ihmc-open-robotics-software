
package us.ihmc.modelFileLoaders.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for material complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="material">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;sequence>
 *         &lt;element name="color" type="{http://www.ros.org}color" minOccurs="0"/>
 *         &lt;element name="texture" type="{http://www.ros.org}texture" minOccurs="0"/>
 *       &lt;/sequence>
 *       &lt;attribute name="name" type="{http://www.w3.org/2001/XMLSchema}string" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "material", namespace = "http://www.ros.org", propOrder = {
    "color",
    "texture"
})
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class URDFMaterial {

    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected URDFColor color;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected URDFTexture texture;
    @XmlAttribute(name = "name")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String name;

    /**
     * Gets the value of the color property.
     * 
     * @return
     *     possible object is
     *     {@link URDFColor }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public URDFColor getColor() {
        return color;
    }

    /**
     * Sets the value of the color property.
     * 
     * @param value
     *     allowed object is
     *     {@link URDFColor }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setColor(URDFColor value) {
        this.color = value;
    }

    /**
     * Gets the value of the texture property.
     * 
     * @return
     *     possible object is
     *     {@link URDFTexture }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public URDFTexture getTexture() {
        return texture;
    }

    /**
     * Sets the value of the texture property.
     * 
     * @param value
     *     allowed object is
     *     {@link URDFTexture }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setTexture(URDFTexture value) {
        this.texture = value;
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

}

/**
 * Set of functions for URDF model saving
 */
#include <sstream>

using std::endl;

std::string getXMLMaterial(urdf::Material mat) {
    std::stringstream s;
    s << "  <material name=\"" << mat.name << "\">" << endl;
    s << "    <color rgba=\"" << mat.color.r << " " << mat.color.g << " " << mat.color.b << " " << mat.color.a << " " << "\"/>" << endl;
    if (!mat.texture_filename.empty())
        s << "    <texture filename=\" " << mat.texture_filename << "\"/>" << endl;
    s << "  </material>" << endl;
    return s.str();
}

std::string getXMLPose(urdf::Pose o) {
    std::stringstream s;
    double roll, pitch, yaw;
    o.rotation.getRPY(roll, pitch, yaw);
    s << "<origin rpy=\"" << roll << " " << pitch << " " << yaw << " " << "\"";
    s << " xyz=\"" << o.position.x << " " << o.position.y << " " << o.position.z << " " << "\"/>";
    return s.str();
}

std::string getXMLInertial(urdf::Inertial o) {
    std::stringstream s;
    s << "  <inertial>" << endl;
    s << "    <mass value=\"" << o.mass << "\"/>" << endl;
    s << "    <inertia ixx = \"" << o.ixx << "\" ixy=\"" << o.ixy << "\" ixz=\"" << o.ixz << "\" iyy=\"" << o.iyy << "\" iyz=\"" << o.iyz << "\" izz=\"" << o.izz << "\"/>" << endl;
    s << "    " << getXMLPose(o.origin) << endl;
    s << "  </inertial>" << endl;
    return s.str();
}

std::string getXMLGeometry(boost::shared_ptr<urdf::Geometry> o) {
    std::stringstream s;
    if (o->type == o->BOX) {
        urdf::Box *b = (urdf::Box*) &(*o);
        s << "<geometry>" << endl;
        s << "  <box size=\"" << b->dim.x << " " << b->dim.y << " " << b->dim.z << " " << "\" />" << endl;
        s << "</geometry>" << endl;
    }

    if (o->type == o->CYLINDER) {
        urdf::Cylinder *c = (urdf::Cylinder*) &(*o);
        s << "<geometry>" << endl;
        s << "  <cylinder radius=\"" << c->radius << "\" length=\"" << c->length << "\" />" << endl;
        s << "</geometry>" << endl;
    }

    if (o->type == o->MESH) {
        urdf::Mesh *m = (urdf::Mesh*) &(*o);
        s << "<geometry>" << endl;
        s << "  <mesh filename=\"" << m->filename << "\" scale=\" " << m->scale.x << " " << m->scale.y << " " << m->scale.z << " " << " \" />" << endl;
        s << "</geometry>" << endl;
    }

    if (o->type == o->SPHERE) {
        urdf::Sphere *sp = (urdf::Sphere*) &(*o);
        s << "<geometry>" << endl;
        s << "  <sphere radius=\"" << sp->radius << "\" />" << endl;
        s << "</geometry>" << endl;
    }
    return s.str();
}

std::string getXMLVisual(urdf::Visual o) {
    std::stringstream s;

    s << "  <visual>" << endl;
    if (o.geometry != NULL)
        s << getXMLGeometry(o.geometry);
    /*if (o.material != NULL)
     s << getXMLMaterial(*o.material);*/
    s << "    <material name=\"" << o.material_name << "\"/>" << endl;
    s << "    " << getXMLPose(o.origin) << endl;
    s << "  </visual>" << endl;

    return s.str();
}

std::string getXMLCollision(urdf::Collision o) {
    std::stringstream s;
    s << "  <collision>" << endl;
    if (o.geometry != NULL)
        s << getXMLGeometry(o.geometry);
    s << "    " << getXMLPose(o.origin) << endl;
    s << "  </collision>" << endl;
    return s.str();
}

std::string getXMLLink(urdf::Link o) {
    std::stringstream s;

    s << "<link name=\"" << o.name << "\">" << endl;
    if (o.inertial != NULL)
        s << getXMLInertial(*o.inertial);
    if (o.visual != NULL)
        s << getXMLVisual(*o.visual);
    if (o.collision != NULL)
        s << getXMLCollision(*o.collision);
    s << "</link>" << endl;

    return s.str();
}

std::string getXMLJoint(urdf::Joint o) {
    std::stringstream s;
    std::string type;
    if (o.type == o.CONTINUOUS)
        type = "continuous";
    else if (o.type == o.UNKNOWN)
        type = "unknown";
    else if (o.type == o.REVOLUTE)
        type = "revolute";
    else if (o.type == o.PRISMATIC)
        type = "prismatic";
    else if (o.type == o.PLANAR)
        type = "planar";
    else if (o.type == o.FLOATING)
        type = "floating";
    else if (o.type == o.FIXED)
        type = "fixed";

    s << "<joint name=\"" << o.name << "\" type=\"" << type << "\">" << endl;
    s << "  " << getXMLPose(o.parent_to_joint_origin_transform) << endl;
    s << "  <axis xyz=\"" << o.axis.x << " " << o.axis.y << " " << o.axis.z << " " << " \"/>" << endl;
    s << "  <parent link=\"" << o.parent_link_name << "\"/>" << endl;
    s << "  <child link=\"" << o.child_link_name << "\"/>" << endl;
    if (o.limits != NULL)
        s << "  <limit  effort=\"" << o.limits->effort << "\" velocity=\"" << o.limits->velocity << "\" lower=\"" << o.limits->lower << "\" upper=\"" << o.limits->upper << "\" />" << endl;
    s << "</joint>" << endl;
    return s.str();
}

std::string getXML(urdf::Model & model) {
    std::stringstream s;

    s << "<?xml version=\"1.0\" ?>" << endl;
    s << "<robot name=\"" << model.getName() << "\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" << endl;
//Print materials
    for (std::map<std::string, boost::shared_ptr<urdf::Material> >::iterator it = model.materials_.begin(); it != model.materials_.end(); ++it) {
        s << getXMLMaterial(*it->second);
    }

//print links
    for (std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator it = model.links_.begin(); it != model.links_.end(); ++it) {
        s << getXMLLink(*it->second);
    }
//print joints
    for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = model.joints_.begin(); it != model.joints_.end(); ++it) {
        s << getXMLJoint(*it->second);
    }

    s << "</robot>";

    return s.str();
}

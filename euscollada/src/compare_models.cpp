/* Author: Yohei Kakiuchi */
#include <ros/ros.h>
#include "collada_parser/collada_parser.h"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"

#include <iostream>
#include <fstream>

using namespace urdf;
using namespace std;

extern bool compare_joints (boost::shared_ptr<const Joint> j0, boost::shared_ptr<const Joint> j1);
boost::shared_ptr<ModelInterface> read_file(string input_file) {
  string xml_string;
  fstream xml_file(input_file.c_str(), fstream::in);
  while (xml_file.good()) {
    string line;
    getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  boost::shared_ptr<ModelInterface> robot;
  if(xml_string.find("<COLLADA") != string::npos) {
    ROS_DEBUG("Parsing robot collada xml string");
    robot = parseCollada(xml_string);
  } else {
    ROS_DEBUG("Parsing robot urdf xml string");
    robot = parseURDF(xml_string);
  }

  if (!robot) {
    cerr << "ERROR: Model Parsing the xml failed / " << input_file << endl;
    robot.reset();
  }
  return robot;
}

#define POSITION_THRESHOLD 10e-6
#define ROTATION_THRESHOLD 10e-5
#define INERTIA_THRESHOLD 10e-4
#define MASS_THRESHOLD 0.01

bool compare_pose (const Pose &p0, const Pose &p1) {
  bool ret = true;
  double x = p0.position.x - p1.position.x;
  double y = p0.position.y - p1.position.y;
  double z = p0.position.z - p1.position.z;
  //x*x + y*y + z*z
  if( x*x + y*y + z*z > POSITION_THRESHOLD ) {
    cerr << "position error" << endl;
    ret = false;
  }

  Rotation rr = p0.rotation * p1.rotation.GetInverse();
  //double r,p,y;
  //r.getRPY(r,p,y);
  //(1.0 - rr.w)
  if ( 1.0 - rr.w > ROTATION_THRESHOLD ) {
    cerr << "rotation error" << endl;
    cerr << 1.0 - rr.w << endl;
    cerr << p0.rotation.x << " " << p0.rotation.y << " " << p0.rotation.z << " " << p0.rotation.w << " / ";
    cerr << p1.rotation.x << " " << p1.rotation.y << " " << p1.rotation.z << " " << p1.rotation.w << " / " << endl;
    cerr << rr.x << " " << rr.y << " " << rr.z << " " << rr.w << endl;
    ret = false;
  }

  return ret;
}

bool compare_inertia(boost::shared_ptr<Inertial> l0, boost::shared_ptr<Inertial> l1) {
  bool ret = true;
  double ixx,ixy,ixz,iyy,iyz,izz;
  ixx = l0->ixx - l1->ixx;
  ixy = l0->ixy - l1->ixy;
  ixz = l0->ixz - l1->ixz;
  iyy = l0->iyy - l1->iyy;
  iyz = l0->iyz - l1->iyz;
  izz = l0->izz - l1->izz;
  if ( ixx*ixx + ixy*ixy + ixz*ixz + iyy*iyy + iyz*iyz + izz*izz > INERTIA_THRESHOLD) ret = false;

  return ret;
}

bool compare_links (boost::shared_ptr<Link> l0, boost::shared_ptr<Link> l1) {
  bool ret = true;
  if(l0->name != l1->name) {
    cerr << "wrong name"  << endl;
  }
  // compare inertial
  if (!!l0->inertial && !!l1->inertial) {
    if ( abs(l0->inertial->mass - l1->inertial->mass) > MASS_THRESHOLD ) {
      cerr << "mass error"  << endl;
      //mass error
      ret = false;
    }
    if (!compare_pose(l0->inertial->origin, l1->inertial->origin)) {
      cerr << "pose error"  << endl;
      ret = false;
    }
    if (!compare_inertia(l0->inertial, l1->inertial)) {
      cerr << "inertia error"  << endl;
      ret = false;
    }
  } else if (!!l0->inertial || !!l1->inertial) {
    cerr << "wrong inertia param"  << endl;
    ret = false;
  }

  //compare_joints(l0->parent_joint, l1->parent_joint);
  // child_joints
  // child_links

  return ret;
}

bool compare_joints (boost::shared_ptr<const Joint> j0, boost::shared_ptr<const Joint> j1) {
  bool ret = true;

  if (j0->name != j1->name) {
    cerr << "wrong name"  << endl;
    ret = false;
  }
  if (j0->child_link_name != j1->child_link_name) {
    cerr << "wrong child_link"  << endl;
    ret = false;
  }
  if (j0->parent_link_name != j1->parent_link_name) {
    cerr << "wrong parent_link"  << endl;
    ret = false;
  }
  if (j0->type != j1->type) {
    cerr << "wrong type"  << endl;
    ret = false;
  }
  if (!compare_pose(j0->parent_to_joint_origin_transform,
                    j1->parent_to_joint_origin_transform)) {
    cerr << "parent_to_joint pose"  << endl;
    ret = false;
  }
  double x,y,z;
  x = j0->axis.x - j1->axis.x;
  y = j0->axis.y - j1->axis.y;
  z = j0->axis.z - j1->axis.z;
  if (x*x + y*y + z*z > POSITION_THRESHOLD) {
    cerr << "axis"  << endl;
    ret = false;
  }

  if (!!j0->limits && !!j1->limits) {
  } else if (!!j0->limits || !!j1->limits) {
    ret = false;
  }
#if 0
  if (!!j0->dynamics && !!j1->dynamics) {
  } else if (!!j0->dynamics || !!j1->dynamics) {
    ret = false;
  }
#endif
  return ret;
}

bool compare_robots (boost::shared_ptr<ModelInterface> r0,
                     boost::shared_ptr<ModelInterface> r1) {
  bool ret = true;
  if (r0->name_ != r1->name_) {
    cerr << "name fail" << endl;
  }

  if (!compare_links(r0->root_link_, r1->root_link_)) {
    ret = false;
  }
  cerr << "moge" << endl;
  if(r0->links_.size() == r1->links_.size()) {
    map<string, boost::shared_ptr<Link> >::iterator it = r0->links_.begin();
    while(it != r0->links_.end()) {
      boost::shared_ptr<Link> lk;
      r1->getLink(it->first, lk);
      if (!lk) {
        cerr << "link not found" << endl;
        ret = false;
      } else {
        if (!compare_links(it->second, lk)) {
          cerr << "compare link fail" << endl;
          ret = false;
        }
      }
      it++;
    }
  } else {
    ret = false;
  }
  cerr << "fuge" << endl;
  if(r0->joints_.size() == r1->joints_.size()) {
    map<string, boost::shared_ptr<Joint> >::iterator it = r0->joints_.begin();
    while(it != r0->joints_.end()) {
      boost::shared_ptr<const Joint> jt;
      jt = r1->getJoint(it->first);
      if (!jt) {
        cerr << "joint not found" << endl;
        ret = false;
      } else {
        if (!compare_joints(it->second, jt)) {
          cerr << "compare joint fail" << endl;
          ret = false;
        }
      }
      it++;
    }
  } else {
    ret = false;
  }

  return ret;
}

int main(int argc, char** argv) {
  if(argc < 2) {
    return -1;
  }
  string fname0(argv[1]);
  string fname1(argv[2]);

  boost::shared_ptr<ModelInterface> r0, r1;

  r0 = read_file(fname0);
  r1 = read_file(fname1);
  if (!r0 || !r1) {
    cerr << "file read error" << endl;
    return -1;
  }

  if(!compare_robots(r0, r1)) {
    cerr << "compare fail!!" << endl;
    return -1;
  }
  cerr << "compare success!!" << endl;
  return 0;
}

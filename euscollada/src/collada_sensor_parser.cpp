/* Author: Yohei Kakiuchi */
#include "collada_sensor_parser.h"

#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <Eigen/Eigen>

bool ColladaSensorParser::getSensorParams (const domExtraRef pextra, ColladaSensorParser::daeSensor &ds) {
  bool get_params = false;
  string sensor_type;
  for (size_t ii = 0; ii < dae.getDatabase()->getElementCount(NULL, "extra", NULL); ii++) {
    domExtra *tmpextra;
    dae.getDatabase()->getElement((daeElement**)&tmpextra, ii, NULL, "extra");
    if (tmpextra->getType() == string("library_sensors")) {
      for (size_t icon = 0; icon < tmpextra->getTechnique_array()[0]->getContents().getCount(); icon++) {
        if ((string("#") + tmpextra->getTechnique_array()[0]->getContents()[icon]->getAttribute("id")) ==
            pextra->getTechnique_array()[0]->getChild("instance_sensor")->getAttribute("url")) {
          sensor_type = tmpextra->getTechnique_array()[0]->getContents()[icon]->getAttribute("type");
          if (sensor_type.size() > 0) {
            get_params = true;
            ds.sensor_type = sensor_type;

            daeTArray< daeSmartRef<daeElement> > elm;
            tmpextra->getTechnique_array()[0]->getContents()[icon]->getChildren(elm);
            for(int i = 0; i < elm.getCount(); i++) {
              daeElementRef ref = elm[i].cast();
              ds.params.push_back(ref);
            }
          }
          break;
        }
      }
    }
  }
  return get_params;
}

domLink* ColladaSensorParser::findLinkfromKinematics (domLink* thisLink, const ColladaSensorParser::LINK_NAME& link_name) {
  if (thisLink->getName()==link_name) return thisLink;
  for(size_t ii = 0; ii < thisLink->getAttachment_full_array().getCount(); ++ii) {
    domLink* tmpLink = findLinkfromKinematics(thisLink->getAttachment_full_array()[ii]->getLink(), link_name);
    if (tmpLink) return tmpLink;
  }
  return NULL;
}

bool ColladaSensorParser::parseSensors (ColladaSensorParser::LINK_NAME_LIST &link_names) {
  int iRet = DAE_OK + 1;
  if(! collada_file.empty() ) {
    iRet = dae.load(collada_file.c_str());
  } else {
    return false;
  }

  if ( dae.getDatabase()->getDocumentCount() != 1 ) {
    // ROS_WARN("Number of documnet is not 1 / %d", dae.getDatabase()->getDocumentCount());
    return false;
  }

  g_document = dae.getDatabase()->getDocument((daeUInt)0);

  if ( dae.getDatabase()->getElementCount(NULL, "articulated_system", NULL) > 0 ) {
    domKinematics_model *thisKinematics;
    dae.getDatabase()->getElement((daeElement**)&thisKinematics, 0, NULL, "kinematics_model");

    for(ColladaSensorParser::LINK_NAME_LIST::iterator link_name = link_names.begin();
        link_name != link_names.end(); link_name++) {

      domLink* thisLink = findLinkfromKinematics(thisKinematics->getTechnique_common()->getLink_array()[0],
                                                 *link_name);

      if (!thisLink) continue;
      domArticulated_system *thisArticulated;
      for ( size_t ii = 0; ii < dae.getDatabase()->getElementCount(NULL, "articulated_system", NULL); ii++) {
        dae.getDatabase()->getElement((daeElement**)&thisArticulated, ii, NULL, "articulated_system");
        if ( thisArticulated->getExtra_array().getCount() > 0 ) break;
      }
      for(size_t ie = 0; ie < thisArticulated->getExtra_array().getCount(); ++ie) {
        domExtraRef pextra = thisArticulated->getExtra_array()[ie];
        // find element which type is attach_sensor and is attached to thisNode
        if ( strcmp(pextra->getType(), "attach_sensor") == 0 ) {

          daeElement* frame_origin = pextra->getTechnique_array()[0]->getChild("frame_origin");
          if ( string(thisKinematics->getId())+string("/")+string(thisLink->getSid()) ==
               frame_origin->getAttribute("link") ) {
            daeSensor dsensor;
            dsensor.name = pextra->getName();
            dsensor.parent_link = *link_name;
            getSensorParams(pextra, dsensor);
            string sensor_url(pextra->getTechnique_array()[0]->getChild("instance_sensor")->getAttribute("url"));
            dsensor.sensor_id = sensor_url.erase(sensor_url.find( "#sensor" ), 7);
            daeTArray<daeElementRef> children;
            frame_origin->getChildren(children);
            for(size_t i = 0; i < children.getCount(); ++i) {
              dsensor.ptrans.push_back(children[i]);
            }
            m_sensors.push_back(dsensor);
            //ROS_WARN_STREAM("Sensor " << pextra->getName() << " is attached to " << link->second->name
            //                << " " << dsensor.sensor_type << " " << sensor_url);
          }
        }
      }
    }
    // sort sensor
    stable_sort(m_sensors.begin(), m_sensors.end(), ColladaSensorParser::daeSensor::compare);
  } else {
    return false;
  }
  return true;
}

void ColladaSensorParser::printURDF(ostream &strm)
{
  strm << "<!-- start of collada_sensor_parser -->" << std::endl;
  for(int i = 0; i < m_sensors.size(); i++) {
    if (m_sensors[i].sensor_type == "base_imu") {
      printURDF_Element_imu(strm, m_sensors[i]);
    } else
    if (m_sensors[i].sensor_type == "base_force6d") {
      printURDF_Element_force(strm, m_sensors[i]);
    } else
    if (m_sensors[i].sensor_type == "base_pinhole_camera") {
      printURDF_Element_camera(strm, m_sensors[i]);
    } else
    if (m_sensors[i].sensor_type == "base_laser1d") {
      printURDF_Element_lidar(strm, m_sensors[i]);
    } else {
      //
    }
  }
  strm << "<!-- end of collada_sensor_parser -->" << std::endl;
}

void getTransfrom(vector<daeElementRef> &transElems, std::string &pos, std::string &rot, bool camera = false)
{
  Eigen::Affine3d trans = Eigen::Affine3d::Identity();
  for (size_t i = 0; i < transElems.size(); i++) {
    domRotateRef protate = daeSafeCast<domRotate>(transElems[i]);
    if ( !! protate ) {
      Eigen::AngleAxisd al(protate->getValue()[3]*(M_PI/180.0),
                           Eigen::Vector3d(protate->getValue()[0], protate->getValue()[1], protate->getValue()[2]));
      Eigen::Affine3d tmp = Eigen::Translation3d(0,0,0) * al;
      trans = trans * tmp;
#if 0
      Eigen::Vector3d vv = trans.translation();
      std::cerr << "pos: " << vv.x() << ", " << vv.y() << ", " << vv.z() << std::endl ;
      Eigen::Quaterniond q(trans.linear());
      Eigen::Matrix3d mat(q);
      std::cout << "mat: " << std::endl;
      std::cout << mat(0, 0) << " " << mat(0, 1) << " " << mat(0, 2) << std::endl
                << mat(1, 0) << " " << mat(1, 1) << " " << mat(1, 2) << std::endl
                << mat(2, 0) << " " << mat(2, 1) << " " << mat(2, 2) << std::endl;
      Eigen::Vector3d rpy = q.matrix().eulerAngles(2, 1, 0);
      std::cerr << "rpy: " << rpy.z() << ", " << rpy.y() << ", " << rpy.x() << std::endl ;
#endif
     } else {
      domTranslateRef ptrans = daeSafeCast<domTranslate>(transElems[i]);
      if ( !! ptrans ) {
        Eigen::Translation3d tr(ptrans->getValue()[0], ptrans->getValue()[1], ptrans->getValue()[2]);
        Eigen::Affine3d tmp = tr * Eigen::Quaterniond::Identity();
        trans = trans * tmp;
#if 0
        Eigen::Vector3d vv = trans.translation();
        std::cerr << "pos: " << vv.x() << ", " << vv.y() << ", " << vv.z() << std::endl ;
        Eigen::Quaterniond q(trans.linear());
        Eigen::Matrix3d mat(q);
        std::cout << "mat: " << std::endl;
        std::cout << mat(0, 0) << " " << mat(0, 1) << " " << mat(0, 2) << std::endl
                  << mat(1, 0) << " " << mat(1, 1) << " " << mat(1, 2) << std::endl
                  << mat(2, 0) << " " << mat(2, 1) << " " << mat(2, 2) << std::endl;
        Eigen::Vector3d rpy = q.matrix().eulerAngles(2, 1, 0);
        std::cerr << "rpy: " << rpy.z() << ", " << rpy.y() << ", " << rpy.x() << std::endl ;
#endif
      }
    }
  }
  Eigen::Vector3d p = trans.translation();
  Eigen::Quaterniond q(trans.linear());
  Eigen::Vector3d rpy = q.matrix().eulerAngles(2, 1, 0);
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(10) << p.x() << " ";
    ss << std::fixed << std::setprecision(10) << p.y() << " ";
    ss << std::fixed << std::setprecision(10) << p.z();
    pos = ss.str();
  }
  {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(10) << rpy.z() << " ";
    ss << std::fixed << std::setprecision(10) << rpy.y() << " ";
    ss << std::fixed << std::setprecision(10) << rpy.x();
    rot = ss.str();
  }
}

void ColladaSensorParser::printURDF_Element_imu   (ostream &strm, ColladaSensorParser::daeSensor &s)
{
  std::string xyz;
  std::string rpy;
  getTransfrom(s.ptrans, xyz, rpy);

  std::string frame = "imu_" + s.name;

  for(int i = 0; i < s.params.size(); i++) {
    std::string ename(s.params[i]->getElementName());
    if ( ename == "max_acceleration" ) {

    } else
    if ( ename == "max_angular_velocity" ) {

    } else {
      //
    }
  }
  //
  strm << "<link name=\"" << frame << "\" />" << std::endl;
  strm << "<joint name=\"" << frame << "_fixed_joint\" type=\"fixed\">" << std::endl;
  strm << "   <parent link=\"" << s.parent_link << "\"/>" << std::endl;
  strm << "   <child link=\"" << frame << "\"/>" << std::endl;
  strm << "   <origin xyz=\"" << xyz << "\" rpy=\"" << rpy << "\"/>" << std::endl;
  strm << "</joint>" << std::endl;
  //
  strm << "<gazebo reference=\"" << frame << "\">" << std::endl;
  strm << "  <gravity>true</gravity>" << std::endl;
  strm << "  <sensor name=\"" << s.name << "\" type=\"imu\">" << std::endl;
  strm << "    <always_on>true</always_on>" << std::endl;
  strm << "    <update_rate></update_rate>" << std::endl;
  strm << "    <visualize>false</visualize>" << std::endl;
  strm << "    <topic>" << (frame + "_topic") << "</topic>" << std::endl;
  strm << "    <plugin filename=\"libgazebo_ros_imu_sensor.so\" name=\"imu_plugin\">" << std::endl;
  strm << "      <bodyName>" << frame << "</bodyName>" << std::endl;
  strm << "      <topicName>" << (frame + "_topic") << "</topicName>" << std::endl;
  strm << "      <updateRateHZ>100</updateRateHZ>" << std::endl;
  strm << "      <gaussianNoise>0.0</gaussianNoise>" << std::endl;
  strm << "      <frameName>" << frame << "</frameName>" << std::endl;
  strm << "      <initialOrientationAsReference>false</initialOrientationAsReference>" << std::endl;
  strm << "    </plugin>" << std::endl;
  strm << "  </sensor>" << std::endl;
  strm << "</gazebo>" << std::endl;
}

void ColladaSensorParser::printURDF_Element_force (ostream &strm, ColladaSensorParser::daeSensor &s)
{
  std::string xyz;
  std::string rpy;
  getTransfrom(s.ptrans, xyz, rpy);

  std::string frame = "force_" + s.name;

  for(int i = 0; i < s.params.size(); i++) {
    std::string ename(s.params[i]->getElementName());
    if ( ename == "load_range_force" ) {

    } else
    if ( ename == "load_range_torque" ) {

    } else {
      //
    }
  }
  //
  //<gazebo reference="${joint_name}">
  //  <provideFeedback>true</provideFeedback>
  //</gazebo>
  //<gazebo>
  //  <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
  //    <updateRate>${rate}</updateRate>
  //    <topicName>${topic}</topicName>
  //    <jointName>${joint_name}</jointName>
  //  </plugin>
  //</gazebo>
  //
  strm << "<gazebo>" << std::endl;
  strm << "   <plugin name=\"contact_sensor_controller\" filename=\"libgazebo_ros_f3d.so\">" << std::endl;
  strm << "      <alwaysOn>true</alwaysOn>" << std::endl;
  strm << "      <updateRate>100.0</updateRate>" << std::endl;
  strm << "      <robotNamespace>/</robotNamespace>" << std::endl;
  strm << "      <bodyName>" << s.parent_link << "</bodyName>" << std::endl;
  strm << "      <topicName>" << frame << "</topicName>" << std::endl;
  strm << "      <frameName>" << s.parent_link << "</frameName>" << std::endl;
  strm << "   </plugin>" << std::endl;
  strm << "</gazebo>" << std::endl;
}

void ColladaSensorParser::printURDF_Element_camera (ostream &strm, ColladaSensorParser::daeSensor &s)
{
  std::string xyz;
  std::string rpy;
  getTransfrom(s.ptrans, xyz, rpy);

  std::string o_xyz;
  std::string o_rpy;
  getTransfrom(s.ptrans, xyz, rpy, true);

  std::string frame = "camera_" + s.name;

  float flength = 0.0;
  float rate = 30.0;
  float fov = 0.0;
  int width = 640;
  int height = 480;
  int pixel_size = 1;
  bool floatq = false;

  for(int i = 0; i < s.params.size(); i++) {
    std::string ename(s.params[i]->getElementName());
    if ( ename  == "measurement_time" ) {
      float tmp = 0.0;
      istringstream ins(s.params[i]->getCharData());
      ins >> tmp;
      rate = 1 / tmp;
    } else
    if ( ename == "focal_length" ) {
      istringstream ins(s.params[i]->getCharData());
      ins >> flength;
    } else
    if ( ename == "format" ) {
      std::string sformat(s.params[i]->getCharData());
      if ( sformat == "uint8" ) {
        floatq = false;
      } else
      if ( sformat == "float32" ) {
        floatq = true;
      }
    } else
    if ( ename == "intrinsic" ) {
      istringstream ins(s.params[i]->getCharData());
      float f0,f1,f2,f3,f4,f5;
      ins >> f0 >> f1 >> f2 >> f3 >> f4 >> f5;

      fov = std::atan( f2 / f0) * 2.0;
    } else
    if ( ename == "image_dimensions" ) {
      istringstream ins(s.params[i]->getCharData());
      ins >> width;
      ins >> height;
      ins >> pixel_size;
    } else {
      //
    }
  }
  //
  strm << "<link name=\"" <<  frame << "_optical\" />" << std::endl;
  strm << "<joint name=\"" << frame << "_optical_fixed_joint\" type=\"fixed\">" << std::endl;
  strm << "  <parent link=\"" << s.parent_link << "\"/>" << std::endl;
  strm << "  <child link=\"" << frame << "_optical\"/>" << std::endl;
  strm << "  <origin xyz=\"" << xyz << "\" rpy=\"" << rpy << "\"/>" << std::endl;
  strm << "</joint>" << std::endl;
  //
  strm << "<link name=\"" <<  frame << "\" />" << std::endl;
  strm << "<joint name=\"" << frame << "_fixed_joint\" type=\"fixed\">" << std::endl;
  strm << "  <parent link=\"" << frame << "_optical\"/>" << std::endl;
  strm << "  <child link=\"" << frame << "\"/>" << std::endl;
  strm << "  <origin xyz=\"0 0 0\" rpy=\"0 -1.570796326795 0\"/>" << std::endl;
  strm << "</joint>" << std::endl;
  //
  strm << "<gazebo reference=\"" << frame << "\">" << std::endl;
  strm << "  <sensor name=\"" << frame << "_camera\" type=\"camera\" >" << std::endl;
  //strm << "    <pose>" << pose << "</pose>" << std::endl;
  strm << "    <always_on>true</always_on>" << std::endl;
  strm << "    <update_rate>" << rate << "</update_rate>" << std::endl;
  strm << "    <visualize>false</visualize>" << std::endl;
  strm << "    <camera name=\"" << frame << "_camera\">" << std::endl;
  strm << "      <horizontal_fov>" << fov << "</horizontal_fov>" << std::endl;
  strm << "      <image>" << std::endl;
  strm << "        <width>"  << width << "</width>" << std::endl;
  strm << "        <height>" << height << "</height>" << std::endl;
  strm << "        <format>B8G8R8</format>" << std::endl;
  strm << "      </image>" << std::endl;
  strm << "      <clip>" << std::endl;
  strm << "        <near>0.02</near>" << std::endl;
  strm << "        <far>300</far>" << std::endl;
  strm << "      </clip>" << std::endl;
  strm << "      <noise>" << std::endl;
  strm << "        <type>gaussian</type>" << std::endl;
  strm << "        <!-- Noise is sampled independently per pixel on each frame." << std::endl;
  strm << "             That pixel's noise value is added to each of its color" << std::endl;
  strm << "             channels, which at that point lie in the range [0,1]. -->" << std::endl;
  strm << "        <mean>0.0</mean>" << std::endl;
  strm << "        <stddev>0.007</stddev>" << std::endl;
  strm << "      </noise>" << std::endl;
  strm << "    </camera>" << std::endl;
  strm << "    <plugin name=\"camera_controller\" filename=\"libgazebo_ros_camera.so\">" << std::endl;
  strm << "      <alwaysOn>true</alwaysOn>" << std::endl;
  strm << "      <updateRate>0.0</updateRate> <!-- ? -->" << std::endl;
  strm << "      <cameraName>" << frame << "_camera</cameraName>" << std::endl;
  strm << "      <imageTopicName>image_raw</imageTopicName>" << std::endl;
  strm << "      <cameraInfoTopicName>camera_info</cameraInfoTopicName>" << std::endl;
  strm << "      <frameName>" << frame << "_optical</frameName>" << std::endl;
  strm << "      <hackBaseline>0.07</hackBaseline>" << std::endl;
  strm << "      <distortionK1>0.0</distortionK1>" << std::endl;
  strm << "      <distortionK2>0.0</distortionK2>" << std::endl;
  strm << "      <distortionK3>0.0</distortionK3>" << std::endl;
  strm << "      <distortionT1>0.0</distortionT1>" << std::endl;
  strm << "      <distortionT2>0.0</distortionT2>" << std::endl;
  strm << "    </plugin>" << std::endl;
  strm << "  </sensor>" << std::endl;
  strm << "</gazebo>" << std::endl;

}
void ColladaSensorParser::printURDF_Element_lidar  (ostream &strm, ColladaSensorParser::daeSensor &s)
{
  // not implemented yet
  for(int i = 0; i < s.params.size(); i++) {
    std::string ename(s.params[i]->getElementName());
    if ( ename == "angle_range" ) {

    } else
    if ( ename == "angle_increment" ) {

    } else
    if ( ename == "measurement_time" ) {

    } else
    if ( ename == "distance_range" ) {

    } else {
      //
    }
  }
}

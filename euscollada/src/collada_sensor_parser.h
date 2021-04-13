/* Author: Yohei Kakiuchi */
#ifndef __COLLADA_SENSOR_PARSER_H__
#define __COLLADA_SENSOR_PARSER_H__

#include <string>
#include <vector>

// using collada just for parsing sensors
#include "dae.h"
#include "dom/domCOLLADA.h"
#ifdef __dom150COLLADA_h__
using namespace ColladaDOM150;
#endif

using namespace std;

#define FLOAT_PRECISION_FINE   "%.16e"
#define FLOAT_PRECISION_COARSE "%.3f"

//
struct ColladaSensorParser {
  typedef string LINK_NAME;
  typedef vector<LINK_NAME> LINK_NAME_LIST;

  struct daeSensor {
    LINK_NAME parent_link;
    string sensor_type;
    string sensor_id;
    string name;
    vector<daeElementRef> ptrans;
    vector<daeElementRef> params;
    static bool compare(const daeSensor& a, const daeSensor& b) {
      return (a.sensor_id < b.sensor_id);
    }
  };

  ColladaSensorParser(const string &fname) {
    collada_file = fname;
  };

  //string getSensorType (const domExtraRef pextra);
  //bool getSensorParams (const domExtraRef pextra, vector<daeElementRef> &params);
  bool getSensorParams (const domExtraRef pextra, daeSensor &ds);
  domLink* findLinkfromKinematics (domLink* thisLink, const LINK_NAME& link_name);
  bool parseSensors(LINK_NAME_LIST &link_names);

  void printURDF(ostream &strm);

  void printURDF_Element_imu   (ostream &strm, daeSensor &s);
  void printURDF_Element_force (ostream &strm, daeSensor &s);
  void printURDF_Element_camera (ostream &strm, daeSensor &s);
  void printURDF_Element_lidar  (ostream &strm, daeSensor &s);

  //
  string collada_file;
  //
  daeDocument *g_document;
  DAE dae;
  vector<daeSensor> m_sensors;

};

#endif//__COLLADA_SENSOR_PARSER_H__

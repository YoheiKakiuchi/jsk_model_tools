/* Author: Yohei Kakiuchi */
#include <ros/ros.h>
#include "collada_parser/collada_parser.h"
#include "urdf/model.h"

#include <iostream>
#include <fstream>

#include "collada_sensor_parser.h"

#include <boost/program_options.hpp>

#include <unistd.h>

using namespace std;
using namespace urdf;

namespace po = boost::program_options;
int main(int argc, char** argv)
{
  // stdout -> stderr
  int tmp_fd = dup(1);
  dup2(2, 1); //1: stdout, 2: stderr

  string input_file;
  string output_file;

  po::options_description desc("Options for collada_to_urdf");
  desc.add_options()
    ("help", "produce help message")
    ("input_file,I", po::value< vector<string> >(), "input file")
    ("output_file,O", po::value< vector<string> >(), "output file")
    ;
  po::positional_options_description p;
  p.add("input_file",  1);
  p.add("output_file", 1);

  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);
  }
  catch (po::error e) {
    cerr << ";; option parse error / " << e.what() << endl;
    return 1;
  }
  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }
  if (vm.count("input_file")) {
    vector<string> aa = vm["input_file"].as< vector<string> >();
    input_file = aa[0];
  }
  if (vm.count("output_file")) {
    vector<string> aa = vm["output_file"].as< vector<string> >();
    output_file = aa[0];
  }

  cerr << ";; Input file is: "
       <<  input_file << endl;
  string xml_string;
  fstream xml_file(input_file.c_str(), fstream::in);
  while ( xml_file.good() )
  {
    string line;
    getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

#if URDFDOM_1_0_0_API
  ModelInterfaceSharedPtr robot;
#else
  boost::shared_ptr<ModelInterface> robot;
#endif

  if( xml_string.find("<COLLADA") != string::npos ) {
    ROS_DEBUG("Parsing robot collada xml string");
    robot = parseCollada(xml_string);
  }
  else
  {
    //ROS_DEBUG("Parsing robot urdf xml string");
    //robot = parseURDF(xml_string);
    //input_file.clear(); // this file is urdf
    return -1;
  }

  if (!robot){
    cerr << "ERROR: Model Parsing the xml failed" << endl;
    return -1;
  }

  // revert stdout -> stderr
  sync();
  dup2(tmp_fd, 1);
  close(tmp_fd);

  ColladaSensorParser sp(input_file);

  ColladaSensorParser::LINK_NAME_LIST ln;
#if URDFDOM_1_0_0_API
  for (map<string, LinkSharedPtr>::iterator link = robot->links_.begin();
#else
  for (map<string, boost::shared_ptr<Link> >::iterator link = robot->links_.begin();
#endif
       link != robot->links_.end(); link++ ) {
    ln.push_back(link->second->name);
  }

  sp.parseSensors(ln);

  if (output_file.size() > 0) {
    ofstream of;
    of.open(output_file.c_str());
    sp.printURDF(of);
  } else {
    sp.printURDF(std::cout);
  }
}

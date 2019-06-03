

#include "LAUNCHER_GUI.hpp"
#include <iostream>


void ParseConfigFile(const std::string& config_filename, std::string& executable_path, std::vector<LAUNCHER_GUI::ENVIRONMENT>& environments)
{
   std::ifstream fin(config_filename.c_str());
   std::string environment;
   std::string dummy;
   int num_rounds;
   
   getline(fin, executable_path);
   while(getline(fin, environment))
   {
      fin >> num_rounds;
      getline(fin, dummy);                   // eat remaining newline char
      environments.push_back(LAUNCHER_GUI::ENVIRONMENT(environment, num_rounds));
   }
}


int main(int argc, char *argv[])
{
  Gtk::Main kit(argc, argv);
  
  std::string executable_path;
  std::vector<LAUNCHER_GUI::ENVIRONMENT> environments;
  ParseConfigFile("shepherd.cfg", executable_path, environments); 
  LAUNCHER_GUI gui(executable_path, environments);
  Gtk::Main::run(gui);

  return 0;
}



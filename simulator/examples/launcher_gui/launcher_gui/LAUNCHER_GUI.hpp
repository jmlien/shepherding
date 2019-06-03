

#ifndef LAUNCHER_GUI_HPP
#define LAUNCHER_GUI_HPP


#include <gtkmm.h>
#include <vector>
#include <string>
#include <fstream>


class LAUNCHER_GUI : public Gtk::Window
{
   public:
   struct ENVIRONMENT
   {
      std::string environment;
      int num_rounds;
      ENVIRONMENT(const std::string& env, const int nr) : environment(env), num_rounds(nr) { }
      ENVIRONMENT(const ENVIRONMENT& env) : environment(env.environment), num_rounds(env.num_rounds) { }
      const ENVIRONMENT& operator =(const ENVIRONMENT& env) { environment = env.environment; num_rounds = env.num_rounds; return *this; }
   };
   
   private:
   Gtk::VBox vbox_;
   Gtk::Entry user_id_entry_;
   Gtk::Button confirm_id_button_;
   Gtk::Label user_id_label_;
   Gtk::Button run_simulation_button_;
   std::string executable_path_;
   std::vector<ENVIRONMENT> environments_;
   int environment_counter_;
   int round_counter_;
   
   protected:
   LAUNCHER_GUI() { }
   LAUNCHER_GUI(const LAUNCHER_GUI&) { }
   void ConfirmIdButton();
   void RunSimulationButton();
   
   public:
   LAUNCHER_GUI(const std::string&, const std::vector<ENVIRONMENT>&);
};


#endif // LAUNCHER_GUI_HPP



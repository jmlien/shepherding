

#include "LAUNCHER_GUI.hpp"
#include <iostream>



LAUNCHER_GUI::LAUNCHER_GUI(const std::string& executable_path, const std::vector<ENVIRONMENT>& environments) :
   confirm_id_button_("Confirm ID"),
   user_id_label_("User ID:"),
   run_simulation_button_("Begin Simulation"),
   executable_path_(executable_path),
   environments_(environments),
   environment_counter_(0),
   round_counter_(0)
{
   this->Gtk::Window::set_size_request(200, 150);
   this->Gtk::Window::set_resizable(false);
   this->Gtk::Window::set_title("Shepherding Simulator");
   this->Gtk::Window::add(vbox_);
   
   user_id_entry_.set_text("Enter your user ID.");
   user_id_entry_.select_region(0, user_id_entry_.get_text_length());
   user_id_entry_.set_max_length(25);
   vbox_.pack_start(user_id_entry_, false, false, 10);
   
   confirm_id_button_.set_size_request(100, 25);
   confirm_id_button_.signal_clicked().connect(sigc::mem_fun(*this, &LAUNCHER_GUI::ConfirmIdButton));
   vbox_.pack_start(confirm_id_button_, false, false, 0);
   
   vbox_.pack_start(user_id_label_, false, false, 5);
   
   run_simulation_button_.signal_clicked().connect(sigc::mem_fun(*this, &LAUNCHER_GUI::RunSimulationButton));
   vbox_.pack_start(run_simulation_button_);
   
   this->Gtk::Window::show_all_children();
   run_simulation_button_.hide();
}

void LAUNCHER_GUI::ConfirmIdButton()
{
   std::string user_id = user_id_entry_.get_text().raw();
   if(user_id != "" && user_id != "Enter your user ID.")
   {
      user_id_label_.set_text("User ID: " + user_id);

	  std::stringstream ss;
	  ss << "Begin Simulation" << "\n\n";
      ss << "Level: " << 1 << " / " << environments_.size() << "\n";
      ss << "Round: " << 1 << " / " << environments_[0].num_rounds;
      run_simulation_button_.set_label(ss.str());

      run_simulation_button_.show();
	  user_id_entry_.set_editable(false);
      confirm_id_button_.hide();
	  user_id_entry_.hide();
   }
}

void LAUNCHER_GUI::RunSimulationButton()
{
   std::string user_id = user_id_label_.get_text().raw();
   user_id = user_id.substr(9, user_id.size() - 9);
   if(user_id != "")
   {
      std::string command = executable_path_ + " -user=" + user_id + " -mouse -disable-autonomy -no-visual-hints " + environments_[environment_counter_].environment;
      
      round_counter_++;
      if(round_counter_ >= environments_[environment_counter_].num_rounds)
      {
         round_counter_ = 0;
         environment_counter_++;
      }


	  iconify();
	  std::cout << "command: " << command << "\n";
      system(command.c_str());
      
      // all environments completed
      if(environment_counter_ == environments_.size())
      {
         exit(0);
      }

      std::stringstream ss;
	  ss << "Next Level" << "\n\n";
      ss << "Level: " << (environment_counter_ + 1) << " / " <<  environments_.size() << "\n";
      ss << "Round: " << (round_counter_ + 1) << " / " << environments_[environment_counter_].num_rounds;
      run_simulation_button_.set_label(ss.str());
      
	  set_keep_above(true);
	  deiconify();
   }
}




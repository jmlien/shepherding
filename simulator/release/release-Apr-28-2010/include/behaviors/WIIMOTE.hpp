

#ifndef WIIMOTE_HPP
#define WIIMOTE_HPP
#if USE_WIIMOTE

#include <wiiuse/wiiuse.h>
#include <string>
#include <cstdlib>
#include <cassert>
#include <vector>


class WIIMOTE
{
   public:
   wiimote_t** wiimote_;
   int reference_count_;
   int* group_reference_count_;
   std::string mac_address_;
   float cursor_x_;
   float cursor_y_;
   
   protected:
   WIIMOTE(wiimote_t** const, const int, const std::string&, const float, const float);
   void Destruct();
   
   public:
   WIIMOTE();
   WIIMOTE(const WIIMOTE&);
   ~WIIMOTE();
   static WIIMOTE Construct();
   static WIIMOTE Construct(const std::string& mac_address);
   static std::vector<WIIMOTE> ConnectMultiple(const int number_of_wiimotes);
   
   float GetCursorX() const;
   float GetCursorY() const;
   std::string ToString() const;
   
   const WIIMOTE& operator =(const WIIMOTE&);
   
   void Update();
};


inline WIIMOTE::WIIMOTE() :
   wiimote_(0),
   reference_count_(0),
   group_reference_count_(&reference_count_)
{
   // do nothing
}

inline WIIMOTE::WIIMOTE(wiimote_t** const wiimote, const int reference_count,
   const std::string& mac_address, const float cursor_x, const float cursor_y) :
   //   
   wiimote_(wiimote),
   reference_count_(reference_count),
   group_reference_count_(&reference_count_),
   mac_address_(mac_address),
   cursor_x_(cursor_x),
   cursor_y_(cursor_y)
{
   // do nothing
}

inline void WIIMOTE::Destruct()
{
   (*group_reference_count_)--;
   if(*group_reference_count_ == 0)
   {
      wiiuse_cleanup(wiimote_, 1);
   }
}   

inline WIIMOTE::WIIMOTE(const WIIMOTE& wiimote) :
   wiimote_(wiimote.wiimote_),
   group_reference_count_(wiimote.group_reference_count_),
   mac_address_(wiimote.mac_address_),
   cursor_x_(wiimote.cursor_x_),
   cursor_y_(wiimote.cursor_y_)
{
   (*group_reference_count_)++;
}

inline WIIMOTE::~WIIMOTE()
{
   Destruct();
}

// it looks like the windows version of wiiuse does not give access to the wiimote's
// mac address, so the empty string is stored as the address instead
#ifdef WIN32
inline WIIMOTE WIIMOTE::Construct()
{
   wiimote_t** wiimote = wiiuse_init(1);
   int found = wiiuse_find(wiimote, 1, 4);
   assert(found > 0);
   wiiuse_connect(wiimote, 1);
   wiiuse_motion_sensing(wiimote[0], 1);
   wiiuse_set_ir(wiimote[0], 1);
   return WIIMOTE(wiimote, 1, "", 0.5f, 0.5f);
}

#else
inline WIIMOTE WIIMOTE::Construct()
{
   wiimote_t** wiimote = wiiuse_init(1);
   int found = wiiuse_find(wiimote, 1, 4);
   assert(found > 0);
   wiiuse_connect(wiimote, 1);
   wiiuse_motion_sensing(wiimote[0], 1);
   wiiuse_set_ir(wiimote[0], 1);
   return WIIMOTE(wiimote, 1, wiimote[0]->bdaddr_str, 0.5f, 0.5f);
}
#endif

// also, windows doesnt allow you to connect to bluetooth devices using 
// a specific address, so instead, it just returns a connection to a random wiimote,
// assuming at least one is found
#ifdef WIN32
inline WIIMOTE WIIMOTE::Construct(const std::string& mac_address)
{
   return WIIMOTE::Construct();
}

// on linux, you can connect to a given device, using its mac address
#else
inline WIIMOTE WIIMOTE::Construct(const std::string& mac_address)
{
   wiimote_t** wiimote = wiiuse_init(1);
   
   // convert mac_address from formatted hex string to array of 6 bytes
   const char* const mac_cstr = mac_address.c_str();
   strcpy((char*)wiimote[0]->bdaddr_str, mac_cstr);
   const char* cursor = mac_cstr;
   for(int i = 0; i < 6; i++)
   {
      int decimal = 0;
      while(*cursor != ':' && *cursor != 0)
      {
         decimal *= 16;
         if(*cursor < 58)
         {
            decimal += *cursor - 48;
         }
         else if(*cursor < 71)
         {
            decimal += *cursor - 55;
         }
         else if(*cursor < 103)
         {
            decimal += *cursor - 87;
         }
         cursor++;
      }
      (unsigned char&)wiimote[0]->bdaddr.b[5 - i] = decimal;
      cursor++;
   }
   (int&)(wiimote[0]->state) = 2049;
 
   wiiuse_connect(wiimote, 1);
   wiiuse_motion_sensing(wiimote[0], 1);
   wiiuse_set_ir(wiimote[0], 1);
   wiiuse_set_ir_sensitivity(wiimote[0], 2); 
   wiiuse_set_smooth_alpha(wiimote[0], 0.75);
   return WIIMOTE(wiimote, 1, mac_address, 0.5f, 0.5f);
}
#endif

inline std::vector<WIIMOTE> WIIMOTE::ConnectMultiple(const int number_of_wiimotes)
{
   wiimote_t** wiimotes = wiiuse_init(number_of_wiimotes);
   int found = wiiuse_find(wiimotes, number_of_wiimotes, 3);
   assert(found >= number_of_wiimotes);
   wiiuse_connect(wiimotes, number_of_wiimotes);
   
   std::vector<WIIMOTE> wiimote_vector;
   wiimote_vector.reserve(number_of_wiimotes);
   for(int i = 0; i < number_of_wiimotes; i++)
   {
       wiiuse_motion_sensing(wiimotes[i], 1);
       wiiuse_set_ir(wiimotes[i], 1);
       wiimote_vector.push_back(WIIMOTE(wiimotes + i, 1, "", 0.5f, 0.5f));
   }
   return wiimote_vector;
}

inline float WIIMOTE::GetCursorX() const
{
   return cursor_x_;
}

inline float WIIMOTE::GetCursorY() const
{
   return cursor_y_;
}

inline const WIIMOTE& WIIMOTE::operator =(const WIIMOTE& wiimote)
{
   Destruct();
   wiimote_ = wiimote.wiimote_;
   group_reference_count_ = wiimote.group_reference_count_;
   (*group_reference_count_)++;
   mac_address_ = wiimote.mac_address_;
   cursor_x_ = wiimote.cursor_x_;
   cursor_y_ = wiimote.cursor_y_;
   return *this;
}

inline void WIIMOTE::Update()
{
   static const float x_scalar = 1.0/1000.0;
   static const float y_scalar = 1.0/750.0;
   wiiuse_poll(wiimote_, 1);
   if(wiimote_[0]->ir.num_dots > 0)
   {
      //cursor_x_ *= 0.5f;
      //cursor_y_ *= 0.5f;
      cursor_x_ = (wiimote_[0]->ir.ax - 50.0)/900.0;
      cursor_y_ = (wiimote_[0]->ir.ay - 40.0)/670.0;
   }
}

#endif // USE_WIIMOTE
#endif // WIIMOTE_HPP



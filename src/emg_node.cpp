#include <emg/emg.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "allegro_hand_emg_grasp_type");
  AHKeyboard allegro_hand_keyboard_cmd;

  signal(SIGINT,quit);

  allegro_hand_keyboard_cmd.keyLoop();

  return(0);
}
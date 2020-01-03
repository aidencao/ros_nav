#ifndef QTAPPNODE_H
#define QTAPPNODE_H
#include <string>

void send_take_off_cmd();
void send_land_cmd();
void send_drone_height_cmd(const float drone_height);
void send_gps_height_and_number(const std::string cmd);

#endif

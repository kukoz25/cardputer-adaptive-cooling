#pragma once

namespace FanRadio {
void begin();
bool ready();
const char* status();
bool send(int button);  // 0 is power; 1, 2, and 3 select a speed.
}

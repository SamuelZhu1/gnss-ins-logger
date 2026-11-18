// Line shell over USB Serial/JTAG. Every reply is one JSON line.
//   status | pose | log start | log stop | mask gnss <s> | cal static | version
#pragma once

namespace shell {
void task(void*);
}

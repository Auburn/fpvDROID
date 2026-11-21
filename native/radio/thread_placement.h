// kepyur radio: how keenly, and on which cores, the OS runs the radio's own threads.
#pragma once

namespace kp {

// Raise the calling thread's priority (the USB stream thread must never miss a chip deadline; the DSP and decode
// threads must keep up). Best effort: logs what the OS allowed.
void boost_thread(const char *what);

// Keep the calling processing thread off the slowest cores: all but the slowest kind, on a chip with faster ones,
// so long as at least two are left. Left to itself, Android now and then runs a thread that is busy half the time
// on a little core (a Pixel 8 Pro's Cortex-A510s, about a third the speed of its A715s): the decode thread was
// there 7 % of its time, falling behind each time, and its queue overflowed in bursts with the thread at half
// load. KP_FAST_CPUS=0 (the "env" debug extra): left to the OS, and the log says so. Linux and Android only.
void keep_off_slow_cores(const char *what);

}  // namespace kp

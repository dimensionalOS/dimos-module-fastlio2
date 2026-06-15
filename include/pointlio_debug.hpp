#ifndef POINTLIO_DEBUG_H_
#define POINTLIO_DEBUG_H_

// Runtime debug flag for FAST-LIO. Default is false → all gated
// informational/diagnostic prints stay silent. Set to true from the
// host application (e.g. the dimos NativeModule wrapper passing a
// `--debug true` CLI flag) when you want the verbose output back.
// True error conditions are not gated and always print.
extern bool pointlio_debug;

#endif

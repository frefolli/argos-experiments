#ifndef SETUP_HH
#define SETUP_HH
/** @file setup.hh */
#include <cstring>

#define PARSE_ENV_SETUP(recipient, value) \
  if (strcmp(strategy, #value) == 0)      \
  {                                       \
    recipient = value;                    \
  }

#define ENABLE_LOG
#endif//SETUP_HH

/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9-dev */

#ifndef PB_COMMUNICATION_PROTO_THRUSTER_COMMAND_PB_H_INCLUDED
#define PB_COMMUNICATION_PROTO_THRUSTER_COMMAND_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _communication_Command {
    double thrust;
    bool emergency_stop;
} communication_Command;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define communication_Command_init_default       {0, 0}
#define communication_Command_init_zero          {0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define communication_Command_thrust_tag         1
#define communication_Command_emergency_stop_tag 2

/* Struct field encoding specification for nanopb */
#define communication_Command_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   thrust,            1) \
X(a, STATIC,   SINGULAR, BOOL,     emergency_stop,    2)
#define communication_Command_CALLBACK NULL
#define communication_Command_DEFAULT NULL

extern const pb_msgdesc_t communication_Command_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define communication_Command_fields &communication_Command_msg

/* Maximum encoded size of messages (where known) */
#define COMMUNICATION_PROTO_THRUSTER_COMMAND_PB_H_MAX_SIZE communication_Command_size
#define communication_Command_size               11

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
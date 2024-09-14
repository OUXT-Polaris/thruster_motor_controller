/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9-dev */

#ifndef PB_COMMUNICATION_PROTO_THRUSTER_COMMAND_PB_H_INCLUDED
#define PB_COMMUNICATION_PROTO_THRUSTER_COMMAND_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _communication_Thrust {
    double thrust;
} communication_Thrust;

typedef struct _communication_EmergencyStop {
    char dummy_field;
} communication_EmergencyStop;

typedef struct _communication_Command {
    pb_size_t which_command;
    union {
        communication_Thrust thrust;
        communication_EmergencyStop emergency_stop;
    } command;
} communication_Command;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define communication_Thrust_init_default        {0}
#define communication_EmergencyStop_init_default {0}
#define communication_Command_init_default       {0, {communication_Thrust_init_default}}
#define communication_Thrust_init_zero           {0}
#define communication_EmergencyStop_init_zero    {0}
#define communication_Command_init_zero          {0, {communication_Thrust_init_zero}}

/* Field tags (for use in manual encoding/decoding) */
#define communication_Thrust_thrust_tag          1
#define communication_Command_thrust_tag         1
#define communication_Command_emergency_stop_tag 2

/* Struct field encoding specification for nanopb */
#define communication_Thrust_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, DOUBLE,   thrust,            1)
#define communication_Thrust_CALLBACK NULL
#define communication_Thrust_DEFAULT NULL

#define communication_EmergencyStop_FIELDLIST(X, a) \

#define communication_EmergencyStop_CALLBACK NULL
#define communication_EmergencyStop_DEFAULT NULL

#define communication_Command_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,thrust,command.thrust),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (command,emergency_stop,command.emergency_stop),   2)
#define communication_Command_CALLBACK NULL
#define communication_Command_DEFAULT NULL
#define communication_Command_command_thrust_MSGTYPE communication_Thrust
#define communication_Command_command_emergency_stop_MSGTYPE communication_EmergencyStop

extern const pb_msgdesc_t communication_Thrust_msg;
extern const pb_msgdesc_t communication_EmergencyStop_msg;
extern const pb_msgdesc_t communication_Command_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define communication_Thrust_fields &communication_Thrust_msg
#define communication_EmergencyStop_fields &communication_EmergencyStop_msg
#define communication_Command_fields &communication_Command_msg

/* Maximum encoded size of messages (where known) */
#define COMMUNICATION_PROTO_THRUSTER_COMMAND_PB_H_MAX_SIZE communication_Command_size
#define communication_Command_size               11
#define communication_EmergencyStop_size         0
#define communication_Thrust_size                9

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

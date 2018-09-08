/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.6-dev at Thu Jul 28 13:34:59 2016. */

#include "dfu-cc.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const bool dfu_init_command_is_debug_default = false;


const pb_field_t dfu_hash_fields[3] = {
    PB_FIELD(  1, UENUM   , REQUIRED, STATIC  , FIRST, dfu_hash_t, hash_type, hash_type, 0),
    PB_FIELD(  2, BYTES   , REQUIRED, STATIC  , OTHER, dfu_hash_t, hash, hash_type, 0),
    PB_LAST_FIELD
};

const pb_field_t dfu_init_command_fields[10] = {
    PB_FIELD(  1, UINT32  , OPTIONAL, STATIC  , FIRST, dfu_init_command_t, fw_version, fw_version, 0),
    PB_FIELD(  2, UINT32  , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, hw_version, fw_version, 0),
    PB_FIELD(  3, UINT32  , REPEATED, STATIC  , OTHER, dfu_init_command_t, sd_req, hw_version, 0),
    PB_FIELD(  4, UENUM   , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, type, sd_req, 0),
    PB_FIELD(  5, UINT32  , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, sd_size, type, 0),
    PB_FIELD(  6, UINT32  , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, bl_size, sd_size, 0),
    PB_FIELD(  7, UINT32  , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, app_size, bl_size, 0),
    PB_FIELD(  8, MESSAGE , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, hash, app_size, &dfu_hash_fields),
    PB_FIELD(  9, BOOL    , OPTIONAL, STATIC  , OTHER, dfu_init_command_t, is_debug, hash, &dfu_init_command_is_debug_default),
    PB_LAST_FIELD
};

const pb_field_t dfu_reset_command_fields[2] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, dfu_reset_command_t, timeout, timeout, 0),
    PB_LAST_FIELD
};

const pb_field_t dfu_command_fields[4] = {
    PB_FIELD(  1, UENUM   , OPTIONAL, STATIC  , FIRST, dfu_command_t, op_code, op_code, 0),
    PB_FIELD(  2, MESSAGE , OPTIONAL, STATIC  , OTHER, dfu_command_t, init, op_code, &dfu_init_command_fields),
    PB_FIELD(  3, MESSAGE , OPTIONAL, STATIC  , OTHER, dfu_command_t, reset, init, &dfu_reset_command_fields),
    PB_LAST_FIELD
};

const pb_field_t dfu_signed_command_fields[4] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, dfu_signed_command_t, command, command, &dfu_command_fields),
    PB_FIELD(  2, UENUM   , REQUIRED, STATIC  , OTHER, dfu_signed_command_t, signature_type, command, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, STATIC  , OTHER, dfu_signed_command_t, signature, signature_type, 0),
    PB_LAST_FIELD
};

const pb_field_t dfu_packet_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, dfu_packet_t, command, command, &dfu_command_fields),
    PB_FIELD(  2, MESSAGE , OPTIONAL, STATIC  , OTHER, dfu_packet_t, signed_command, command, &dfu_signed_command_fields),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 *
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(dfu_init_command_t, hash) < 65536 && pb_membersize(dfu_command_t, init) < 65536 && pb_membersize(dfu_command_t, reset) < 65536 && pb_membersize(dfu_signed_command_t, command) < 65536 && pb_membersize(dfu_packet_t, command) < 65536 && pb_membersize(dfu_packet_t, signed_command) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_dfu_hash_dfu_init_command_dfu_reset_command_dfu_command_dfu_signed_command_dfu_packet)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 *
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(dfu_init_command_t, hash) < 256 && pb_membersize(dfu_command_t, init) < 256 && pb_membersize(dfu_command_t, reset) < 256 && pb_membersize(dfu_signed_command_t, command) < 256 && pb_membersize(dfu_packet_t, command) < 256 && pb_membersize(dfu_packet_t, signed_command) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_dfu_hash_dfu_init_command_dfu_reset_command_dfu_command_dfu_signed_command_dfu_packet)
#endif


/* @@protoc_insertion_point(eof) */

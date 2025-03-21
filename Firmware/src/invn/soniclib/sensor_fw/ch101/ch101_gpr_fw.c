//
// Chirp Microsystems Firmware Header Generator v2.4 (Python 3.8.5)
//
// File generated at 2022-03-07 17:41:08.553367 by klong
// Script input parameters:
//   - Input file:                 invn.chirpmicro.asic.ch101.gpr.v43a.hex
//   - Output file:                ch101_gpr_fw.c
//   - Part number:                101
//   - Program size:               2048
//   - DMEM start address:         0x0
//   - PMEM start address:         0xf800
//   - Firmware name:              gpr
//   - Firmware name (sanitized):  gpr
//   - Firmware git version:       gpr_gpr-101_v43a
//   - Firmware git sha1:          b94d9640060b12396993f62109fd0e9076d981f7
//
// Copyright (c) 2022, Chirp Microsystems. All rights reserved.
//

#include <stdint.h>
#include "ch101.h"
#include "ch101_gpr.h"

const char * ch101_gpr_version = "gpr_gpr_gpr-101_v43a";
const char * ch101_gpr_gitsha1 = "b94d9640060b12396993f62109fd0e9076d981f7";

#define RAM_INIT_ADDRESS 2420
#define RAM_INIT_WRITE_SIZE   13

uint16_t get_ch101_gpr_fw_ram_init_addr(void) { return (uint16_t)RAM_INIT_ADDRESS;}
uint16_t get_ch101_gpr_fw_ram_init_size(void) { return (uint16_t)RAM_INIT_WRITE_SIZE;}

const unsigned char ram_ch101_gpr_init[RAM_INIT_WRITE_SIZE] = {
0x06, 0x00, 0x00, 0x00, 0x00, 0xFA, 0x00, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00, };

const unsigned char * get_ram_ch101_gpr_init_ptr(void) { return &ram_ch101_gpr_init[0];}

#define	CH101_GPR_TEXT_SIZE	1956
#define	CH101_GPR_VEC_SIZE	32

const uint16_t  ch101_gpr_text_size	= CH101_GPR_TEXT_SIZE;
const uint16_t  ch101_gpr_vec_size	= CH101_GPR_VEC_SIZE;

const unsigned char ch101_gpr_fw_text[CH101_GPR_TEXT_SIZE] = {
0x0a, 0x12, 0x09, 0x12, 0x08, 0x12, 0x07, 0x12, 0x06, 0x12, 0x05, 0x12, 0x04, 0x12, 0x31, 0x82, 
0x81, 0x4d, 0x06, 0x00, 0xc2, 0x43, 0x6a, 0x09, 0x07, 0x43, 0x05, 0x43, 0x09, 0x43, 0x0a, 0x43, 
0x0c, 0x93, 0x9e, 0x24, 0x04, 0x4c, 0x36, 0x40, 0xdc, 0x05, 0x0b, 0x4a, 0x0b, 0x5b, 0x1f, 0x41, 
0x06, 0x00, 0x4f, 0x93, 0x90, 0x20, 0x38, 0x40, 0xa0, 0x05, 0x08, 0x5b, 0x3b, 0x90, 0x12, 0x00, 
0x02, 0x20, 0x36, 0x40, 0x58, 0x02, 0x3b, 0x90, 0x46, 0x00, 0x02, 0x20, 0x36, 0x40, 0xc8, 0x00, 
0x3e, 0x40, 0x1c, 0x02, 0x0f, 0x4b, 0x0f, 0x5f, 0x0f, 0x5e, 0x2c, 0x4f, 0x0f, 0x4b, 0x1f, 0x53, 
0x0f, 0x5f, 0x0e, 0x5f, 0x2d, 0x4e, 0xb0, 0x12, 0xe4, 0xfe, 0x88, 0x4c, 0x00, 0x00, 0x3a, 0x90, 
0x16, 0x00, 0x11, 0x2c, 0xf2, 0x90, 0x20, 0x00, 0x01, 0x02, 0x0d, 0x24, 0x3e, 0x40, 0x24, 0x09, 
0x0e, 0x5b, 0x0f, 0x4c, 0x2f, 0x8e, 0x0f, 0x93, 0x02, 0x34, 0x3f, 0xe3, 0x1f, 0x53, 0x88, 0x4f, 
0x00, 0x00, 0x8e, 0x4c, 0x00, 0x00, 0x5f, 0x42, 0x12, 0x02, 0x0a, 0x9f, 0x2c, 0x2c, 0x0e, 0x4a, 
0x0e, 0x5e, 0x3f, 0x40, 0x62, 0x07, 0x0f, 0x5e, 0x3e, 0x50, 0xa0, 0x05, 0x2d, 0x4f, 0x81, 0x4d, 
0x00, 0x00, 0x28, 0x4e, 0x0c, 0x4d, 0xb0, 0x12, 0xf2, 0xfd, 0x08, 0x9c, 0x12, 0x28, 0x5e, 0x42, 
0x13, 0x02, 0x4d, 0x4e, 0x2c, 0x41, 0xb0, 0x12, 0xb4, 0xfd, 0x81, 0x4c, 0x00, 0x00, 0x4e, 0x4e, 
0x0c, 0x48, 0x0d, 0x4e, 0xb0, 0x12, 0xb4, 0xfd, 0x2c, 0x5f, 0x2c, 0x81, 0x8f, 0x4c, 0x00, 0x00, 
0x02, 0x3c, 0xaf, 0x4e, 0x00, 0x00, 0x2f, 0x4f, 0x0f, 0x56, 0x3f, 0x90, 0x89, 0x13, 0x04, 0x28, 
0x3f, 0x40, 0x88, 0x13, 0x01, 0x3c, 0x0f, 0x46, 0x05, 0x93, 0x2f, 0x20, 0x5e, 0x42, 0x11, 0x02, 
0x0e, 0x9a, 0x2b, 0x2c, 0x08, 0x4a, 0x08, 0x58, 0x38, 0x50, 0xa0, 0x05, 0x2f, 0x98, 0x03, 0x28, 
0x07, 0x93, 0x06, 0x20, 0x22, 0x3c, 0x07, 0x93, 0x03, 0x20, 0x81, 0x4a, 0x04, 0x00, 0x17, 0x43, 
0x3f, 0x40, 0x1c, 0x02, 0x0e, 0x4b, 0x0e, 0x5e, 0x0e, 0x5f, 0x2c, 0x4e, 0x1b, 0x53, 0x0b, 0x5b, 
0x0b, 0x5f, 0x2d, 0x4b, 0xb0, 0x12, 0x00, 0xfe, 0x88, 0x4c, 0x00, 0x00, 0x0c, 0x99, 0x02, 0x28, 
0x09, 0x4c, 0x0b, 0x3c, 0x0f, 0x4a, 0x1f, 0x81, 0x04, 0x00, 0x1f, 0x83, 0x81, 0x4f, 0x02, 0x00, 
0x07, 0x43, 0x15, 0x43, 0x02, 0x3c, 0x8b, 0x43, 0x62, 0x07, 0x1a, 0x53, 0x14, 0x83, 0x65, 0x23, 
0x07, 0x93, 0x05, 0x20, 0x05, 0x93, 0x07, 0x20, 0xb2, 0x43, 0x18, 0x02, 0x4b, 0x3c, 0x1a, 0x81, 
0x04, 0x00, 0x81, 0x4a, 0x02, 0x00, 0x82, 0x49, 0x1a, 0x02, 0x1a, 0x41, 0x04, 0x00, 0x4a, 0x4a, 
0x12, 0xc3, 0x09, 0x10, 0x18, 0x41, 0x02, 0x00, 0x88, 0x11, 0x38, 0x90, 0xfd, 0xff, 0x20, 0x38, 
0x46, 0x4a, 0x06, 0x58, 0x06, 0x56, 0x37, 0x40, 0xa0, 0x05, 0x07, 0x56, 0x08, 0x93, 0x0f, 0x34, 
0x3e, 0x40, 0x1c, 0x02, 0x0f, 0x46, 0x0f, 0x5f, 0x0f, 0x5e, 0x2c, 0x4f, 0x0f, 0x46, 0x1f, 0x53, 
0x0f, 0x5f, 0x0e, 0x5f, 0x2d, 0x4e, 0xb0, 0x12, 0x00, 0xfe, 0x87, 0x4c, 0x00, 0x00, 0x87, 0x99, 
0x00, 0x00, 0x06, 0x28, 0x26, 0x83, 0x27, 0x83, 0x18, 0x83, 0x38, 0x90, 0xfd, 0xff, 0xe6, 0x37, 
0x48, 0x5a, 0x0f, 0x48, 0x0f, 0x5f, 0x1e, 0x4f, 0xa2, 0x05, 0x1f, 0x4f, 0xa0, 0x05, 0x09, 0x8f, 
0x09, 0x59, 0x0e, 0x8f, 0x3d, 0x42, 0x4f, 0x43, 0x4f, 0x5f, 0x0e, 0x99, 0x02, 0x2c, 0x09, 0x8e, 
0x5f, 0x53, 0x09, 0x59, 0x1d, 0x83, 0xf8, 0x23, 0x48, 0x48, 0x88, 0x10, 0x4f, 0x4f, 0x08, 0xdf, 
0x82, 0x48, 0x18, 0x02, 0x31, 0x52, 0x30, 0x40, 0x74, 0xff, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 
0x0c, 0x12, 0x0b, 0x12, 0xd2, 0xc3, 0x7f, 0x09, 0xc2, 0x93, 0x14, 0x02, 0x3b, 0x20, 0x1b, 0x43, 
0x1c, 0x42, 0x2e, 0x02, 0x1d, 0x42, 0x2c, 0x02, 0xb0, 0x12, 0xe4, 0xfe, 0x1c, 0x92, 0x7a, 0x09, 
0x1a, 0x28, 0x1f, 0x42, 0x2e, 0x02, 0x0f, 0x11, 0x0f, 0x11, 0x1f, 0x82, 0x2c, 0x02, 0x1f, 0x93, 
0x02, 0x38, 0x3f, 0x43, 0x01, 0x3c, 0x1f, 0x43, 0xc2, 0x93, 0x7c, 0x09, 0x07, 0x24, 0x5e, 0x42, 
0x7c, 0x09, 0x8e, 0x11, 0x0f, 0x9e, 0x02, 0x24, 0x0b, 0x43, 0x02, 0x3c, 0x82, 0x5f, 0x0e, 0x02, 
0xc2, 0x4f, 0x7c, 0x09, 0x0f, 0x3c, 0xb2, 0x50, 0x14, 0x00, 0x0e, 0x02, 0xb2, 0x90, 0x2d, 0x01, 
0x0e, 0x02, 0x06, 0x28, 0xb2, 0x80, 0xc8, 0x00, 0x0e, 0x02, 0x12, 0xc3, 0x12, 0x10, 0x7a, 0x09, 
0xc2, 0x43, 0x7c, 0x09, 0x0b, 0x93, 0x3c, 0x20, 0xd2, 0x43, 0x14, 0x02, 0xb2, 0x40, 0x1e, 0x3f, 
0x6c, 0x09, 0x36, 0x3c, 0xd2, 0x93, 0x14, 0x02, 0x31, 0x20, 0xf2, 0x90, 0x03, 0x00, 0x72, 0x09, 
0x06, 0x24, 0xc2, 0x93, 0x72, 0x09, 0x19, 0x24, 0xd2, 0x83, 0x72, 0x09, 0x16, 0x3c, 0xf2, 0x90, 
0x20, 0x00, 0x01, 0x02, 0x12, 0x24, 0x1c, 0x42, 0x2e, 0x02, 0x1d, 0x42, 0x2c, 0x02, 0xb0, 0x12, 
0xe4, 0xfe, 0x82, 0x9c, 0x78, 0x09, 0x05, 0x28, 0x82, 0x4c, 0x78, 0x09, 0x92, 0x53, 0x74, 0x09, 
0x04, 0x3c, 0xe2, 0x43, 0x72, 0x09, 0x92, 0x83, 0x74, 0x09, 0xe2, 0x93, 0x72, 0x09, 0x0b, 0x24, 
0xc2, 0x93, 0x72, 0x09, 0x0d, 0x20, 0xe2, 0x43, 0x14, 0x02, 0xe2, 0xd3, 0x7f, 0x09, 0xb2, 0x40, 
0x80, 0x10, 0xd0, 0x01, 0x05, 0x3c, 0xd2, 0x43, 0x01, 0x02, 0x02, 0x3c, 0x82, 0x43, 0xf0, 0x01, 
0xf2, 0x90, 0x03, 0x00, 0x72, 0x09, 0x06, 0x2c, 0x5c, 0x42, 0x07, 0x02, 0x5d, 0x42, 0x72, 0x09, 
0xb0, 0x12, 0x00, 0xf8, 0x92, 0x42, 0x0e, 0x02, 0xf0, 0x01, 0xe2, 0x93, 0x14, 0x02, 0x12, 0x28, 
0xd2, 0xd3, 0xe0, 0x01, 0xd2, 0xc3, 0xe0, 0x01, 0xd2, 0xb3, 0x7f, 0x09, 0x10, 0x20, 0xb2, 0x40, 
0x77, 0x06, 0xa6, 0x01, 0x3c, 0x40, 0x3c, 0x00, 0xb0, 0x12, 0x84, 0xff, 0xb2, 0x40, 0x77, 0x01, 
0xa6, 0x01, 0x05, 0x3c, 0x5c, 0x43, 0xb0, 0x12, 0x2e, 0xfc, 0xa2, 0xc2, 0x92, 0x01, 0xa2, 0xd2, 
0x92, 0x01, 0xd2, 0x42, 0x70, 0x09, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 
0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0a, 0x12, 0xb2, 0x40, 0x80, 0x5a, 
0x20, 0x01, 0xe2, 0x42, 0xe0, 0x01, 0xd2, 0x43, 0xe2, 0x01, 0xf2, 0x40, 0x40, 0x00, 0x01, 0x02, 
0xf2, 0x40, 0x3c, 0x00, 0x07, 0x02, 0xb2, 0x40, 0x64, 0x00, 0x0e, 0x02, 0xc2, 0x43, 0x00, 0x02, 
0xd2, 0x43, 0x05, 0x02, 0xc2, 0x43, 0x11, 0x02, 0xb2, 0x40, 0x00, 0x01, 0x02, 0x02, 0xf2, 0x40, 
0x03, 0x00, 0xc2, 0x01, 0xb2, 0x40, 0x00, 0x02, 0xa6, 0x01, 0xb2, 0x40, 0x00, 0x06, 0xa6, 0x01, 
0xb2, 0x40, 0x1c, 0x02, 0xb0, 0x01, 0x3f, 0x40, 0x07, 0x00, 0x82, 0x4f, 0xb2, 0x01, 0xb2, 0x40, 
0x77, 0x01, 0xa6, 0x01, 0xb2, 0x40, 0x00, 0x01, 0x90, 0x01, 0x82, 0x4f, 0x92, 0x01, 0x0a, 0x43, 
0x02, 0x3c, 0x32, 0xd0, 0x58, 0x00, 0x5f, 0x42, 0x01, 0x02, 0x0a, 0x9f, 0x20, 0x24, 0x5a, 0x42, 
0x01, 0x02, 0x0f, 0x4a, 0x3f, 0x80, 0x10, 0x00, 0x18, 0x24, 0x3f, 0x80, 0x10, 0x00, 0x15, 0x24, 
0x3f, 0x80, 0x20, 0x00, 0x0d, 0x20, 0xc2, 0x43, 0x14, 0x02, 0xe2, 0x42, 0x72, 0x09, 0xb2, 0x40, 
0x1e, 0x18, 0x6c, 0x09, 0x92, 0x42, 0x0e, 0x02, 0xf0, 0x01, 0x5c, 0x43, 0xb0, 0x12, 0x2e, 0xfc, 
0xe2, 0x42, 0x70, 0x09, 0xe2, 0xc3, 0xe0, 0x01, 0x02, 0x3c, 0xe2, 0xd3, 0xe0, 0x01, 0x32, 0xc2, 
0x03, 0x43, 0xc2, 0x93, 0x7f, 0x09, 0xd5, 0x27, 0x32, 0xd0, 0x18, 0x00, 0xd4, 0x3f, 0xd2, 0xd3, 
0x7f, 0x09, 0xf2, 0x90, 0x40, 0x00, 0x01, 0x02, 0x2f, 0x24, 0xd2, 0x92, 0x07, 0x02, 0x76, 0x09, 
0x30, 0x24, 0xd2, 0x42, 0x07, 0x02, 0x76, 0x09, 0x5f, 0x42, 0x07, 0x02, 0x3f, 0x80, 0x0b, 0x00, 
0xc2, 0x93, 0x72, 0x09, 0x04, 0x20, 0xb2, 0x40, 0x58, 0x38, 0x50, 0x09, 0x03, 0x3c, 0xb2, 0x40, 
0x58, 0x24, 0x50, 0x09, 0x3b, 0x40, 0xf8, 0x4f, 0x3d, 0x40, 0x52, 0x09, 0x5e, 0x43, 0x3f, 0xb0, 
0x80, 0xff, 0x0b, 0x20, 0x0f, 0x5f, 0x0f, 0x5f, 0x0f, 0x5f, 0x3f, 0x50, 0x00, 0x4c, 0x8d, 0x4f, 
0x00, 0x00, 0x5e, 0x53, 0xc2, 0x4e, 0x73, 0x09, 0x0c, 0x3c, 0x2d, 0x53, 0x8d, 0x4b, 0xfe, 0xff, 
0x5e, 0x53, 0x3f, 0x80, 0x7f, 0x00, 0xeb, 0x3f, 0xb2, 0x40, 0x40, 0x20, 0x50, 0x09, 0xd2, 0x43, 
0x73, 0x09, 0x4c, 0x93, 0x04, 0x20, 0xb2, 0x40, 0x82, 0x10, 0xa2, 0x01, 0x03, 0x3c, 0xb2, 0x40, 
0x86, 0x10, 0xa2, 0x01, 0x5f, 0x42, 0x73, 0x09, 0x0f, 0x93, 0x06, 0x24, 0x3e, 0x40, 0x50, 0x09, 
0xb2, 0x4e, 0xa4, 0x01, 0x1f, 0x83, 0xfc, 0x23, 0x92, 0x42, 0x6c, 0x09, 0xa0, 0x01, 0xc2, 0x93, 
0x14, 0x02, 0x03, 0x24, 0xb2, 0xd0, 0x10, 0x00, 0xa2, 0x01, 0x92, 0x43, 0xae, 0x01, 0xa2, 0x43, 
0xae, 0x01, 0x30, 0x41, 0x0f, 0x12, 0x5f, 0x42, 0x80, 0x09, 0x0f, 0x93, 0x15, 0x24, 0x1f, 0x83, 
0x26, 0x24, 0x1f, 0x83, 0x29, 0x20, 0xb2, 0x90, 0x16, 0x00, 0x6e, 0x09, 0x07, 0x2c, 0x1f, 0x42, 
0x6e, 0x09, 0xdf, 0x42, 0xc1, 0x01, 0x00, 0x02, 0x92, 0x53, 0x6e, 0x09, 0xd2, 0x83, 0x71, 0x09, 
0x1b, 0x20, 0xc2, 0x43, 0x80, 0x09, 0x18, 0x3c, 0x5f, 0x42, 0xc1, 0x01, 0x82, 0x4f, 0x6e, 0x09, 
0xd2, 0x43, 0x80, 0x09, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0x3f, 0x90, 0x06, 0x00, 0x0c, 0x20, 
0xf2, 0x40, 0x24, 0x00, 0xe0, 0x01, 0xb2, 0x40, 0x03, 0x00, 0xd8, 0x01, 0x05, 0x3c, 0xd2, 0x42, 
0xc1, 0x01, 0x71, 0x09, 0xe2, 0x43, 0x80, 0x09, 0xf2, 0xd0, 0x10, 0x00, 0xc2, 0x01, 0xf2, 0xd0, 
0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 
0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 0x92, 0x42, 0x02, 0x02, 0x90, 0x01, 0xe2, 0x93, 
0x01, 0x02, 0x03, 0x20, 0xd2, 0x83, 0x7e, 0x09, 0x0e, 0x24, 0xd2, 0xb3, 0x7f, 0x09, 0x11, 0x20, 
0xb2, 0x40, 0x77, 0x06, 0xa6, 0x01, 0x3c, 0x40, 0x3c, 0x00, 0xb0, 0x12, 0x84, 0xff, 0xb2, 0x40, 
0x77, 0x01, 0xa6, 0x01, 0x06, 0x3c, 0xd2, 0x42, 0x05, 0x02, 0x7e, 0x09, 0x5c, 0x43, 0xb0, 0x12, 
0x2e, 0xfc, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 0x3e, 0x41, 
0x3f, 0x41, 0x00, 0x13, 0x3d, 0xf0, 0x0f, 0x00, 0x3d, 0xe0, 0x0f, 0x00, 0x0d, 0x5d, 0x0d, 0x5d, 
0x00, 0x5d, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 
0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 
0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 
0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x12, 0xc3, 0x0c, 0x10, 0x30, 0x41, 
0x0a, 0x12, 0x1d, 0x93, 0x03, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x02, 0x3c, 0x3c, 0xe3, 0x1c, 0x53, 
0x0e, 0x4d, 0x0f, 0x4c, 0x0e, 0x11, 0x0f, 0x11, 0x0b, 0x43, 0x0c, 0x4e, 0x0d, 0x4b, 0xb0, 0x12, 
0xb8, 0xfe, 0x0a, 0x4c, 0x0c, 0x4f, 0x0d, 0x4b, 0xb0, 0x12, 0xb8, 0xfe, 0x1f, 0x93, 0x03, 0x34, 
0x0e, 0x8c, 0x0f, 0x5a, 0x02, 0x3c, 0x0e, 0x5c, 0x0f, 0x8a, 0x1b, 0x53, 0x2b, 0x92, 0xed, 0x3b, 
0x0c, 0x4e, 0x3a, 0x41, 0x30, 0x41, 0x0f, 0x12, 0x0e, 0x12, 0x0d, 0x12, 0x0c, 0x12, 0x0b, 0x12, 
0xe2, 0xb3, 0xe0, 0x01, 0x12, 0x24, 0xd2, 0x42, 0xe0, 0x01, 0x70, 0x09, 0xe2, 0xc3, 0xe0, 0x01, 
0xa2, 0xc2, 0x92, 0x01, 0x4c, 0x43, 0xf2, 0x90, 0x20, 0x00, 0x01, 0x02, 0x01, 0x24, 0x5c, 0x43, 
0xb0, 0x12, 0x2e, 0xfc, 0xb1, 0xc0, 0xf0, 0x00, 0x0a, 0x00, 0x3b, 0x41, 0x3c, 0x41, 0x3d, 0x41, 
0x3e, 0x41, 0x3f, 0x41, 0x00, 0x13, 0x0f, 0x12, 0xc2, 0x43, 0x80, 0x09, 0x92, 0x53, 0x6e, 0x09, 
0xb2, 0x90, 0xa0, 0x03, 0x6e, 0x09, 0x03, 0x28, 0x82, 0x43, 0x6e, 0x09, 0x05, 0x3c, 0x1f, 0x42, 
0x6e, 0x09, 0xd2, 0x4f, 0x00, 0x02, 0xc0, 0x01, 0xf2, 0xd0, 0x20, 0x00, 0xc2, 0x01, 0xb1, 0xc0, 
0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 0x00, 0x13, 0x3d, 0xf0, 0x0f, 0x00, 0x3d, 0xe0, 0x0f, 0x00, 
0x0d, 0x5d, 0x00, 0x5d, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 
0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 0x0c, 0x11, 
0x0c, 0x11, 0x30, 0x41, 0x1c, 0x93, 0x02, 0x34, 0x3c, 0xe3, 0x1c, 0x53, 0x0f, 0x4c, 0x1d, 0x93, 
0x02, 0x34, 0x3d, 0xe3, 0x1d, 0x53, 0x0c, 0x4d, 0x0c, 0x9f, 0x03, 0x2c, 0x0e, 0x4c, 0x0c, 0x4f, 
0x0f, 0x4e, 0x12, 0xc3, 0x0f, 0x10, 0x0f, 0x11, 0x0c, 0x5f, 0x30, 0x41, 0x0f, 0x12, 0xb2, 0xf0, 
0xef, 0xff, 0xa2, 0x01, 0x3f, 0x40, 0x00, 0x28, 0x1f, 0x52, 0x74, 0x09, 0x82, 0x4f, 0xa0, 0x01, 
0xb1, 0xc0, 0xf0, 0x00, 0x02, 0x00, 0x3f, 0x41, 0x00, 0x13, 0x92, 0x42, 0xda, 0x01, 0x0a, 0x02, 
0x82, 0x43, 0xd8, 0x01, 0xe2, 0x42, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x13, 
0x31, 0x40, 0x00, 0x0a, 0xb0, 0x12, 0x9a, 0xff, 0x0c, 0x43, 0xb0, 0x12, 0x6a, 0xfb, 0xb0, 0x12, 
0x9e, 0xff, 0xe2, 0xc3, 0x7f, 0x09, 0x92, 0x42, 0xd2, 0x01, 0x16, 0x02, 0xb1, 0xc0, 0xf0, 0x00, 
0x00, 0x00, 0x00, 0x13, 0xd2, 0xd3, 0xe0, 0x01, 0xe2, 0xc2, 0xe0, 0x01, 0xb1, 0xc0, 0xf0, 0x00, 
0x00, 0x00, 0x00, 0x13, 0x34, 0x41, 0x35, 0x41, 0x36, 0x41, 0x37, 0x41, 0x38, 0x41, 0x39, 0x41, 
0x3a, 0x41, 0x30, 0x41, 0x1c, 0x83, 0x03, 0x43, 0xfd, 0x23, 0x30, 0x41, 0xb1, 0xc0, 0xf0, 0x00, 
0x00, 0x00, 0x00, 0x13, 0x32, 0xd0, 0x10, 0x00, 0xfd, 0x3f, 0x1c, 0x43, 0x30, 0x41, 0x03, 0x43, 
0xff, 0x3f, 0x00, 0x13, };

const unsigned char ch101_gpr_fw_vec[CH101_GPR_VEC_SIZE] = {
0x86, 0xfe, 0xe4, 0xfc, 0xa2, 0xff, 0x2a, 0xff, 0x46, 0xfe, 0x00, 0x00, 0x94, 0xff, 0x0a, 0xfa, 
0x0c, 0xff, 0x8c, 0xff, 0x64, 0xff, 0x00, 0x00, 0x52, 0xff, 0x5e, 0xfd, 0x94, 0xff, 0x40, 0xff, 
};


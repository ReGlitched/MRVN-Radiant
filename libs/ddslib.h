/* -----------------------------------------------------------------------------

   DDS Library

   Based on code from Nvidia's DDS example:
   http://www.nvidia.com/object/dxtc_decompression_code.html

   Copyright (c) 2003 Randy Reddig
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   Redistributions of source code must retain the above copyright notice, this list
   of conditions and the following disclaimer.

   Redistributions in binary form must reproduce the above copyright notice, this
   list of conditions and the following disclaimer in the documentation and/or
   other materials provided with the distribution.

   Neither the names of the copyright holders nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   ----------------------------------------------------------------------------- */


#pragma once


/* dependencies */
#include <stdio.h>
#include <memory.h>



/* c++ marker */
#ifdef __cplusplus
extern "C"
{
#endif



/* dds definition */
typedef enum
{
	DDS_PF_ARGB8888,
	DDS_PF_DXT1,
	DDS_PF_DXT2,
	DDS_PF_DXT3,
	DDS_PF_DXT4,
	DDS_PF_DXT5,
	DDS_PF_BC4,
	DDS_PF_BC5,
	DDS_PF_BC6H,
	DDS_PF_BC7,
	DDS_PF_UNKNOWN
}
ddsPF_t;


/* DX10 extended header (follows standard DDS header when fourCC == "DX10") */
typedef struct ddsDX10Header_s
{
	unsigned int dxgiFormat;
	unsigned int resourceDimension;
	unsigned int miscFlag;
	unsigned int arraySize;
	unsigned int miscFlags2;
}
ddsDX10Header_t;

/* DXGI format values we care about */
#define DXGI_FORMAT_BC1_TYPELESS          70
#define DXGI_FORMAT_BC1_UNORM             71
#define DXGI_FORMAT_BC1_UNORM_SRGB        72
#define DXGI_FORMAT_BC2_TYPELESS          73
#define DXGI_FORMAT_BC2_UNORM             74
#define DXGI_FORMAT_BC2_UNORM_SRGB        75
#define DXGI_FORMAT_BC3_TYPELESS          76
#define DXGI_FORMAT_BC3_UNORM             77
#define DXGI_FORMAT_BC3_UNORM_SRGB        78
#define DXGI_FORMAT_BC4_TYPELESS          79
#define DXGI_FORMAT_BC4_UNORM             80
#define DXGI_FORMAT_BC4_SNORM             81
#define DXGI_FORMAT_BC5_TYPELESS          82
#define DXGI_FORMAT_BC5_UNORM             83
#define DXGI_FORMAT_BC5_SNORM             84
#define DXGI_FORMAT_BC6H_TYPELESS         94
#define DXGI_FORMAT_BC6H_UF16             95
#define DXGI_FORMAT_BC6H_SF16             96
#define DXGI_FORMAT_BC7_TYPELESS          97
#define DXGI_FORMAT_BC7_UNORM             98
#define DXGI_FORMAT_BC7_UNORM_SRGB        99


/* 16bpp stuff */
#define DDS_LOW_5       0x001F;
#define DDS_MID_6       0x07E0;
#define DDS_HIGH_5      0xF800;
#define DDS_MID_555     0x03E0;
#define DDS_HI_555      0x7C00;


/* structures */
typedef struct ddsColorKey_s
{
	unsigned int colorSpaceLowValue;
	unsigned int colorSpaceHighValue;
}
ddsColorKey_t;


typedef struct ddsCaps_s
{
	unsigned int caps1;
	unsigned int caps2;
	unsigned int caps3;
	unsigned int caps4;
}
ddsCaps_t;


typedef struct ddsMultiSampleCaps_s
{
	unsigned short flipMSTypes;
	unsigned short bltMSTypes;
}
ddsMultiSampleCaps_t;


typedef struct ddsPixelFormat_s
{
	unsigned int size;
	unsigned int flags;
	unsigned int fourCC;
	union
	{
		unsigned int rgbBitCount;
		unsigned int yuvBitCount;
		unsigned int zBufferBitDepth;
		unsigned int alphaBitDepth;
		unsigned int luminanceBitCount;
		unsigned int bumpBitCount;
		unsigned int privateFormatBitCount;
	};
	union
	{
		unsigned int rBitMask;
		unsigned int yBitMask;
		unsigned int stencilBitDepth;
		unsigned int luminanceBitMask;
		unsigned int bumpDuBitMask;
		unsigned int operations;
	};
	union
	{
		unsigned int gBitMask;
		unsigned int uBitMask;
		unsigned int zBitMask;
		unsigned int bumpDvBitMask;
		ddsMultiSampleCaps_t multiSampleCaps;
	};
	union
	{
		unsigned int bBitMask;
		unsigned int vBitMask;
		unsigned int stencilBitMask;
		unsigned int bumpLuminanceBitMask;
	};
	union
	{
		unsigned int rgbAlphaBitMask;
		unsigned int yuvAlphaBitMask;
		unsigned int luminanceAlphaBitMask;
		unsigned int rgbZBitMask;
		unsigned int yuvZBitMask;
	};
}
ddsPixelFormat_t;


typedef struct ddsBuffer_s
{
	/* magic: 'dds ' */
	char magic[ 4 ];

	/* directdraw surface */
	unsigned int size;
	unsigned int flags;
	unsigned int height;
	unsigned int width;
	union
	{
		int pitch;
		unsigned int linearSize;
	};
	unsigned int backBufferCount;
	union
	{
		unsigned int mipMapCount;
		unsigned int refreshRate;
		unsigned int srcVBHandle;
	};
	unsigned int alphaBitDepth;
	unsigned int reserved;
	unsigned int surface; // void *surface;
	union
	{
		ddsColorKey_t ckDestOverlay;
		unsigned int emptyFaceColor;
	};
	ddsColorKey_t ckDestBlt;
	ddsColorKey_t ckSrcOverlay;
	ddsColorKey_t ckSrcBlt;
	union
	{
		ddsPixelFormat_t pixelFormat;
		unsigned int fvf;
	};
	ddsCaps_t ddsCaps;
	unsigned int textureStage;

	/* data (Varying size) */
	unsigned char data[ 4 ];
}
ddsBuffer_t;


typedef struct ddsColorBlock_s
{
	unsigned short colors[ 2 ];
	unsigned char row[ 4 ];
}
ddsColorBlock_t;


typedef struct ddsAlphaBlockExplicit_s
{
	unsigned short row[ 4 ];
}
ddsAlphaBlockExplicit_t;


typedef struct ddsAlphaBlock3BitLinear_s
{
	unsigned char alpha0;
	unsigned char alpha1;
	unsigned char stuff[ 6 ];
}
ddsAlphaBlock3BitLinear_t;


typedef struct ddsColor_s
{
	unsigned char r, g, b, a;
}
ddsColor_t;



/* public functions */
int                     DDSGetInfo( ddsBuffer_t *dds, int *width, int *height, ddsPF_t *pf );
int                     DDSDecompress( ddsBuffer_t *dds, unsigned char *pixels );



#ifdef __cplusplus
}
#endif

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



/* dependencies */
#include "ddslib.h"

#define BCDEC_IMPLEMENTATION
#include "bcdec/bcdec.h"

#include <math.h>



/* endian tomfoolery */
typedef union
{
	float f;
	char c[ 4 ];
}
floatSwapUnion;


#ifndef __BIG_ENDIAN__
	#ifdef _SGI_SOURCE
		#define __BIG_ENDIAN__
	#endif
#endif


#ifdef __BIG_ENDIAN__

int   DDSBigLong( int src ) { return src; }
short DDSBigShort( short src ) { return src; }
float DDSBigFloat( float src ) { return src; }

int DDSLittleLong( int src ){
	return ( ( src & 0xFF000000 ) >> 24 ) |
		   ( ( src & 0x00FF0000 ) >> 8 ) |
		   ( ( src & 0x0000FF00 ) << 8 ) |
		   ( ( src & 0x000000FF ) << 24 );
}

short DDSLittleShort( short src ){
	return ( ( src & 0xFF00 ) >> 8 ) |
		   ( ( src & 0x00FF ) << 8 );
}

float DDSLittleFloat( float src ){
	floatSwapUnion in,out;
	in.f = src;
	out.c[ 0 ] = in.c[ 3 ];
	out.c[ 1 ] = in.c[ 2 ];
	out.c[ 2 ] = in.c[ 1 ];
	out.c[ 3 ] = in.c[ 0 ];
	return out.f;
}

#else /*__BIG_ENDIAN__*/

int   DDSLittleLong( int src ) { return src; }
short DDSLittleShort( short src ) { return src; }
float DDSLittleFloat( float src ) { return src; }

int DDSBigLong( int src ){
	return ( ( src & 0xFF000000 ) >> 24 ) |
		   ( ( src & 0x00FF0000 ) >> 8 ) |
		   ( ( src & 0x0000FF00 ) << 8 ) |
		   ( ( src & 0x000000FF ) << 24 );
}

short DDSBigShort( short src ){
	return ( ( src & 0xFF00 ) >> 8 ) |
		   ( ( src & 0x00FF ) << 8 );
}

float DDSBigFloat( float src ){
	floatSwapUnion in,out;
	in.f = src;
	out.c[ 0 ] = in.c[ 3 ];
	out.c[ 1 ] = in.c[ 2 ];
	out.c[ 2 ] = in.c[ 1 ];
	out.c[ 3 ] = in.c[ 0 ];
	return out.f;
}

#endif /*__BIG_ENDIAN__*/



/*
   DDSDecodePixelFormat()
   determines which pixel format the dds texture is in
 */

static void DDSDecodePixelFormat( ddsBuffer_t *dds, ddsPF_t *pf ){
	unsigned int fourCC;


	/* dummy check */
	if ( dds == NULL || pf == NULL ) {
		return;
	}

	/* extract fourCC */
	fourCC = dds->pixelFormat.fourCC;

	/* test it */
	if ( fourCC == 0 ) {
		*pf = DDS_PF_ARGB8888;
	}
	else if ( fourCC == *( (unsigned int*) "DXT1" ) ) {
		*pf = DDS_PF_DXT1;
	}
	else if ( fourCC == *( (unsigned int*) "DXT2" ) ) {
		*pf = DDS_PF_DXT2;
	}
	else if ( fourCC == *( (unsigned int*) "DXT3" ) ) {
		*pf = DDS_PF_DXT3;
	}
	else if ( fourCC == *( (unsigned int*) "DXT4" ) ) {
		*pf = DDS_PF_DXT4;
	}
	else if ( fourCC == *( (unsigned int*) "DXT5" ) ) {
		*pf = DDS_PF_DXT5;
	}
	else if ( fourCC == *( (unsigned int*) "ATI1" ) ) {
		*pf = DDS_PF_BC4;
	}
	else if ( fourCC == *( (unsigned int*) "ATI2" ) ) {
		*pf = DDS_PF_BC5;
	}
	else if ( fourCC == *( (unsigned int*) "BC4U" ) ) {
		*pf = DDS_PF_BC4;
	}
	else if ( fourCC == *( (unsigned int*) "BC5U" ) ) {
		*pf = DDS_PF_BC5;
	}
	else if ( fourCC == *( (unsigned int*) "DX10" ) ) {
		/* DX10 extended header follows the standard DDS header */
		ddsDX10Header_t *dx10 = (ddsDX10Header_t *)( &dds->data[0] );
		unsigned int dxgiFormat = dx10->dxgiFormat;

		if ( dxgiFormat == DXGI_FORMAT_BC1_TYPELESS
		  || dxgiFormat == DXGI_FORMAT_BC1_UNORM
		  || dxgiFormat == DXGI_FORMAT_BC1_UNORM_SRGB ) {
			*pf = DDS_PF_DXT1;
		}
		else if ( dxgiFormat == DXGI_FORMAT_BC2_TYPELESS
		       || dxgiFormat == DXGI_FORMAT_BC2_UNORM
		       || dxgiFormat == DXGI_FORMAT_BC2_UNORM_SRGB ) {
			*pf = DDS_PF_DXT3;
		}
		else if ( dxgiFormat == DXGI_FORMAT_BC3_TYPELESS
		       || dxgiFormat == DXGI_FORMAT_BC3_UNORM
		       || dxgiFormat == DXGI_FORMAT_BC3_UNORM_SRGB ) {
			*pf = DDS_PF_DXT5;
		}
		else if ( dxgiFormat == DXGI_FORMAT_BC4_TYPELESS
		       || dxgiFormat == DXGI_FORMAT_BC4_UNORM
		       || dxgiFormat == DXGI_FORMAT_BC4_SNORM ) {
			*pf = DDS_PF_BC4;
		}
		else if ( dxgiFormat == DXGI_FORMAT_BC5_TYPELESS
		       || dxgiFormat == DXGI_FORMAT_BC5_UNORM
		       || dxgiFormat == DXGI_FORMAT_BC5_SNORM ) {
			*pf = DDS_PF_BC5;
		}
		else if ( dxgiFormat == DXGI_FORMAT_BC6H_TYPELESS
		       || dxgiFormat == DXGI_FORMAT_BC6H_UF16
		       || dxgiFormat == DXGI_FORMAT_BC6H_SF16 ) {
			*pf = DDS_PF_BC6H;
		}
		else if ( dxgiFormat == DXGI_FORMAT_BC7_TYPELESS
		       || dxgiFormat == DXGI_FORMAT_BC7_UNORM
		       || dxgiFormat == DXGI_FORMAT_BC7_UNORM_SRGB ) {
			*pf = DDS_PF_BC7;
		}
		else {
			*pf = DDS_PF_UNKNOWN;
		}
	}
	else{
		*pf = DDS_PF_UNKNOWN;
	}
}



/*
   DDSGetDataOffset()
   returns the offset to pixel data, accounting for DX10 extended header
 */

static unsigned char* DDSGetDataPtr( ddsBuffer_t *dds ){
	unsigned int fourCC = dds->pixelFormat.fourCC;
	if ( fourCC == *( (unsigned int*) "DX10" ) ) {
		/* skip the 20-byte DX10 extended header */
		return dds->data + sizeof( ddsDX10Header_t );
	}
	return dds->data;
}


/*
   DDSGetInfo()
   extracts relevant info from a dds texture, returns 0 on success
 */

int DDSGetInfo( ddsBuffer_t *dds, int *width, int *height, ddsPF_t *pf ){
	/* dummy test */
	if ( dds == NULL ) {
		return -1;
	}

	/* test dds header */
	if ( *( (int*) dds->magic ) != *( (int*) "DDS " ) ) {
		return -1;
	}
	if ( DDSLittleLong( dds->size ) != 124 ) {
		return -1;
	}

	/* extract width and height */
	if ( width != NULL ) {
		*width = DDSLittleLong( dds->width );
	}
	if ( height != NULL ) {
		*height = DDSLittleLong( dds->height );
	}

	/* get pixel format */
	DDSDecodePixelFormat( dds, pf );

	/* return ok */
	return 0;
}



/*
   DDSDecompressBlockFormat()
   generic block decompression using bcdec, outputs RGBA8888
 */

static int DDSDecompressBlockFormat( ddsBuffer_t *dds, int width, int height, unsigned char *pixels, ddsPF_t pf ){
	int x, y, xBlocks, yBlocks;
	int blockSize;
	unsigned char *src;
	unsigned char block[4 * 4 * 4]; /* 4x4 pixels, 4 bytes each (RGBA) */

	xBlocks = ( width + 3 ) / 4;
	yBlocks = ( height + 3 ) / 4;

	src = DDSGetDataPtr( dds );

	switch ( pf ) {
	case DDS_PF_DXT1:   blockSize = BCDEC_BC1_BLOCK_SIZE; break;
	case DDS_PF_DXT2:
	case DDS_PF_DXT3:   blockSize = BCDEC_BC2_BLOCK_SIZE; break;
	case DDS_PF_DXT4:
	case DDS_PF_DXT5:   blockSize = BCDEC_BC3_BLOCK_SIZE; break;
	case DDS_PF_BC4:    blockSize = BCDEC_BC4_BLOCK_SIZE; break;
	case DDS_PF_BC5:    blockSize = BCDEC_BC5_BLOCK_SIZE; break;
	case DDS_PF_BC6H:   blockSize = BCDEC_BC6H_BLOCK_SIZE; break;
	case DDS_PF_BC7:    blockSize = BCDEC_BC7_BLOCK_SIZE; break;
	default: return -1;
	}

	for ( y = 0; y < yBlocks; y++ ) {
		for ( x = 0; x < xBlocks; x++ ) {
			int bx, by;
			int pixW = ( x * 4 + 4 <= width )  ? 4 : width  - x * 4;
			int pixH = ( y * 4 + 4 <= height ) ? 4 : height - y * 4;

			switch ( pf ) {
			case DDS_PF_DXT1:
				bcdec_bc1( src, block, 4 * 4 );
				break;
			case DDS_PF_DXT2:
			case DDS_PF_DXT3:
				bcdec_bc2( src, block, 4 * 4 );
				break;
			case DDS_PF_DXT4:
			case DDS_PF_DXT5:
				bcdec_bc3( src, block, 4 * 4 );
				break;
			case DDS_PF_BC4:
				/* BC4: single channel (R), expand to RGBA */
				{
					unsigned char rBlock[4 * 4];
					bcdec_bc4( src, rBlock, 4 );
					for ( by = 0; by < 4; by++ ) {
						for ( bx = 0; bx < 4; bx++ ) {
							int idx = ( by * 4 + bx ) * 4;
							unsigned char r = rBlock[ by * 4 + bx ];
							block[idx + 0] = r;
							block[idx + 1] = r;
							block[idx + 2] = r;
							block[idx + 3] = 255;
						}
					}
				}
				break;
			case DDS_PF_BC5:
				/* BC5: two channels (RG), expand to RGBA */
				{
					unsigned char rgBlock[4 * 4 * 2];
					bcdec_bc5( src, rgBlock, 4 * 2 );
					for ( by = 0; by < 4; by++ ) {
						for ( bx = 0; bx < 4; bx++ ) {
							int idx = ( by * 4 + bx ) * 4;
							int srcIdx = ( by * 4 + bx ) * 2;
							block[idx + 0] = rgBlock[srcIdx + 0];
							block[idx + 1] = rgBlock[srcIdx + 1];
							block[idx + 2] = 0;
							block[idx + 3] = 255;
						}
					}
				}
				break;
			case DDS_PF_BC6H:
				/* BC6H: HDR float, convert to 8-bit RGBA */
				{
					float rgbBlock[4 * 4 * 3];
					bcdec_bc6h_float( src, rgbBlock, 4 * 3, 0 );
					for ( by = 0; by < 4; by++ ) {
						for ( bx = 0; bx < 4; bx++ ) {
							int idx = ( by * 4 + bx ) * 4;
							int srcIdx = ( by * 4 + bx ) * 3;
							/* simple tonemap: clamp to [0,1] */
							float r = rgbBlock[srcIdx + 0];
							float g = rgbBlock[srcIdx + 1];
							float b = rgbBlock[srcIdx + 2];
							if ( r < 0.0f ) r = 0.0f; if ( r > 1.0f ) r = 1.0f;
							if ( g < 0.0f ) g = 0.0f; if ( g > 1.0f ) g = 1.0f;
							if ( b < 0.0f ) b = 0.0f; if ( b > 1.0f ) b = 1.0f;
							block[idx + 0] = (unsigned char)( r * 255.0f + 0.5f );
							block[idx + 1] = (unsigned char)( g * 255.0f + 0.5f );
							block[idx + 2] = (unsigned char)( b * 255.0f + 0.5f );
							block[idx + 3] = 255;
						}
					}
				}
				break;
			case DDS_PF_BC7:
				bcdec_bc7( src, block, 4 * 4 );
				break;
			default:
				return -1;
			}

			/* copy decompressed block into output image */
			for ( by = 0; by < pixH; by++ ) {
				for ( bx = 0; bx < pixW; bx++ ) {
					int dstIdx = ( ( y * 4 + by ) * width + ( x * 4 + bx ) ) * 4;
					int srcIdx = ( by * 4 + bx ) * 4;
					pixels[dstIdx + 0] = block[srcIdx + 0];
					pixels[dstIdx + 1] = block[srcIdx + 1];
					pixels[dstIdx + 2] = block[srcIdx + 2];
					pixels[dstIdx + 3] = block[srcIdx + 3];
				}
			}

			src += blockSize;
		}
	}

	return 0;
}


/*
   DDSDecompressARGB8888()
   decompresses an argb 8888 format texture
 */

static int DDSDecompressARGB8888( ddsBuffer_t *dds, int width, int height, unsigned char *pixels ){
	int x, y;
	unsigned char               *in, *out;


	/* setup */
	in = DDSGetDataPtr( dds );
	out = pixels;

	/* walk y */
	for ( y = 0; y < height; y++ )
	{
		/* walk x */
		for ( x = 0; x < width; x++ )
		{
			*out++ = *in++;
			*out++ = *in++;
			*out++ = *in++;
			*out++ = *in++;
		}
	}

	/* return ok */
	return 0;
}



/*
   DDSDecompress()
   decompresses a dds texture into an rgba image buffer, returns 0 on success
 */

int DDSDecompress( ddsBuffer_t *dds, unsigned char *pixels ){
	int width, height, r;
	ddsPF_t pf;


	/* get dds info */
	r = DDSGetInfo( dds, &width, &height, &pf );
	if ( r ) {
		return r;
	}

	/* decompress */
	switch ( pf )
	{
	case DDS_PF_ARGB8888:
		r = DDSDecompressARGB8888( dds, width, height, pixels );
		break;

	case DDS_PF_DXT1:
	case DDS_PF_DXT2:
	case DDS_PF_DXT3:
	case DDS_PF_DXT4:
	case DDS_PF_DXT5:
	case DDS_PF_BC4:
	case DDS_PF_BC5:
	case DDS_PF_BC6H:
	case DDS_PF_BC7:
		r = DDSDecompressBlockFormat( dds, width, height, pixels, pf );
		break;

	default:
	case DDS_PF_UNKNOWN:
		memset( pixels, 0xFF, width * height * 4 );
		r = -1;
		break;
	}

	/* return to sender */
	return r;
}

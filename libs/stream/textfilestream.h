/*
   Copyright (C) 2001-2006, William Joseph.
   All Rights Reserved.

   This file is part of GtkRadiant.

   GtkRadiant is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   GtkRadiant is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GtkRadiant; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#pragma once

#include "itextstream.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>

/// \brief A wrapper around a file input stream opened for reading in text mode. Similar to std::ifstream.
class TextFileInputStream : public TextInputStream
{
	FILE* m_file;
public:
	TextFileInputStream( const char* name ){
		m_file = name[0] == '\0' ? 0 : fopen( name, "rt" );
	}
	~TextFileInputStream(){
		if ( !failed() ) {
			fclose( m_file );
		}
	}

	bool failed() const {
		return m_file == 0;
	}

	std::size_t read( char* buffer, std::size_t length ){
		return fread( buffer, 1, length, m_file );
	}
};

/// \brief A TextInputStream that pre-reads an entire file into memory for fast parsing.
/// Eliminates per-read I/O overhead when parsing large files.
class TextBufferInputStream : public TextInputStream
{
	std::vector<char> m_buffer;
	std::size_t m_pos;
	bool m_failed;
public:
	TextBufferInputStream( const char* name ) : m_pos( 0 ), m_failed( true ){
		if ( name[0] == '\0' ) {
			return;
		}
		FILE* file = fopen( name, "rb" );
		if ( file == 0 ) {
			return;
		}
		fseek( file, 0, SEEK_END );
		const long size = ftell( file );
		if ( size <= 0 ) {
			fclose( file );
			if ( size == 0 ) {
				m_failed = false; // empty file is valid
			}
			return;
		}
		fseek( file, 0, SEEK_SET );
		m_buffer.resize( static_cast<std::size_t>( size ) );
		const std::size_t bytesRead = fread( m_buffer.data(), 1, m_buffer.size(), file );
		m_buffer.resize( bytesRead );
		fclose( file );
		m_failed = false;
	}

	bool failed() const {
		return m_failed;
	}

	std::size_t read( char* buffer, std::size_t length ){
		const std::size_t remaining = m_buffer.size() - m_pos;
		const std::size_t count = std::min( length, remaining );
		if ( count > 0 ) {
			std::memcpy( buffer, m_buffer.data() + m_pos, count );
			m_pos += count;
		}
		return count;
	}
};

/// \brief A wrapper around a file input stream opened for writing in text mode. Similar to std::ofstream.
class TextFileOutputStream : public TextOutputStream
{
	FILE* m_file;
public:
	TextFileOutputStream( const char* name ){
		m_file = name[0] == '\0' ? 0 : fopen( name, "wt" );
	}
	~TextFileOutputStream(){
		if ( !failed() ) {
			fclose( m_file );
		}
	}

	bool failed() const {
		return m_file == 0;
	}

	std::size_t write( const char* buffer, std::size_t length ){
		return fwrite( buffer, 1, length, m_file );
	}
};

template<typename T>
inline TextFileOutputStream& operator<<( TextFileOutputStream& ostream, const T& t ){
	return ostream_write( ostream, t );
}

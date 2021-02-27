/*!
 * \file SickLMS1xxBufferMonitor.cc
 * \brief Implements a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS 1xx LIDAR.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

/* Auto-generated header */
#include <sicktoolbox/SickConfig.hh>

/* Implementation dependencies */
#include <iostream>
#include <sys/ioctl.h>

#include <sicktoolbox/SickLMS1xxBufferMonitor.hh>
#include <sicktoolbox/SickLMS1xxMessage.hh>
#include <sicktoolbox/SickLMS1xxUtility.hh>
#include <sicktoolbox/SickException.hh>

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   */
  SickLMS1xxBufferMonitor::SickLMS1xxBufferMonitor( ) : SickBufferMonitor< SickLMS1xxBufferMonitor, SickLMS1xxMessage >(this) { }

  /**
   * \brief Acquires the next message from the SickLMS1xx byte stream
   * \param &sick_message The returned message object
   */
  void SickLMS1xxBufferMonitor::GetNextMessageFromDataStream( SickLMS1xxMessage &sick_message ) throw( SickIOException ) {

    /* Flush the input buffer */
    uint8_t byte_buffer = 0;
    uint8_t payload_buffer[SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};
    
    try {

      /* Flush the TCP receive buffer */
      _flushTCPRecvBuffer();

      /* Search for STX in the byte stream */
      do {
	
 	/* Grab the next byte from the stream */
 	_readBytes(&byte_buffer,1,DEFAULT_SICK_LMS_1XX_BYTE_TIMEOUT);
	
      }
      while (byte_buffer != 0x02);
      
      /* Ok, now acquire the payload! (until ETX) */
      int payload_length = 0;
      do {
	
	payload_length++;
 	_readBytes(&payload_buffer[payload_length-1],1,DEFAULT_SICK_LMS_1XX_BYTE_TIMEOUT);
	
      }
      while (payload_buffer[payload_length-1] != 0x03);
      payload_length--;
      
      /* Build the return message object based upon the received payload
       * NOTE: In constructing this message we ignore the header bytes
       *       buffered since the BuildMessage routine will insert the
       *       correct header automatically and verify the message size
       */
      sick_message.BuildMessage(payload_buffer,payload_length);

      /* Success */
      
    }
    
    catch(SickTimeoutException &sick_timeout) { /* This is ok! */ }
    
    /* Catch any serious IO buffer exceptions */
    catch(SickIOException &sick_io_exception) {
      throw;
    }
    
    /* A sanity check */
    catch (...) {
      throw;
    }
    
  }

  /**
   * \brief Flushes TCP receive buffer contents
   */
  void SickLMS1xxBufferMonitor::_flushTCPRecvBuffer( ) const throw (SickIOException) {
    
    char null_byte;
    int num_bytes_waiting = 0;    

    /* Acquire number of awaiting bytes */
    if (ioctl(_sick_fd,FIONREAD,&num_bytes_waiting)) {
      throw SickIOException("SickLMS1xxBufferMonitor::_flushTCPRecvBuffer: ioctl() failed!");
    }
    
    /* Flush awaiting bytes */
    for (int i = 0; i < num_bytes_waiting; i++) {
      
      /* Capture a single byte from the stream! */
      if (read(_sick_fd,&null_byte,1) != 1) {
	throw SickIOException("SickLMS1xxBufferMonitor::_flushTCPRecvBuffer: ioctl() failed!");
      }	  
      
    }
    
  }
  
  /**
   * \brief A standard destructor
   */
  SickLMS1xxBufferMonitor::~SickLMS1xxBufferMonitor( ) { }
    
} /* namespace SickToolbox */

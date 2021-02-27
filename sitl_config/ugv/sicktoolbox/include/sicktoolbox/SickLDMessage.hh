/*!
 * \file SickLDMessage.hh
 * \brief Defines the class SickLDMessage.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#ifndef SICK_LD_MESSAGE_HH
#define SICK_LD_MESSAGE_HH

/* Definition dependencies */
#include <string.h>
#include <arpa/inet.h>
#include "SickMessage.hh"

#define SICK_LD_MSG_HEADER_LEN             (8)  ///< Sick LD message header length in bytes
#define SICK_LD_MSG_PAYLOAD_MAX_LEN     (5816)  ///< Sick LD maximum payload length
#define SICK_LD_MSG_TRAILER_LEN            (1)  ///< Sick LD length of the message trailer

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \class SickLDMessage
   * \brief A class to represent all messages sent to and from the Sick LD unit.
   */
  class SickLDMessage : public SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN > {
  
  public:
    
    /** A standard constructor */
    SickLDMessage( );
    
    /** Constructs a packet by using BuildMessage */
    SickLDMessage( const uint8_t * const payload_buffer, const unsigned int payload_length );
    
    /** Constructs a packet using ParseMessage() */
    SickLDMessage( const uint8_t * const message_buffer );
    
    /** Construct a well-formed raw packet */
    void BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length );
    
    /** Populates fields from a (well-formed) raw packet */
    void ParseMessage( const uint8_t * const message_buffer );
    
    /** Get the length of the service code associated with the message */
    uint8_t GetServiceCode( ) const { return _message_buffer[8]; }
    
    /** Get the service sub-code associated with the message */
    uint8_t GetServiceSubcode( ) const { return _message_buffer[9]; }
    
    /** Get the checksum for the packet */
    uint8_t GetChecksum( ) const { return _message_buffer[_message_length-1]; }
    
    /** A debugging function that prints the contents of the frame. */
    void Print( ) const;
    
    /** Destructor */
    ~SickLDMessage( );

  private:
    
    /** Computes the checksum of the frame.
     *  NOTE: Uses XOR of single bytes over packet payload data.
     */
    uint8_t _computeXOR( const uint8_t * const data, const uint32_t length );
    
  };
  
} /* namespace SickToolbox */

#endif /* SICK_LD_MESSAGE_HH */

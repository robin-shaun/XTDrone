/*!
 * \file SickLDMessage.cc
 * \brief Implements the class SickLDMessage.
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

/* Auto-generated header */
#include <sicktoolbox/SickConfig.hh>

/* Implementation dependencies */
#include <iomanip>
#include <iostream>
#include <arpa/inet.h> 

#include <sicktoolbox/SickLDMessage.hh>
#include <sicktoolbox/SickLDUtility.hh> // for byye-order conversions where necessary

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A default constructor
   */
  SickLDMessage::SickLDMessage( ) :
    SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN >()  {

    /* Initialize the object */
    Clear(); 
  }
  
  /**
   * \brief Another constructor.
   * \param *payload_buffer The payload for the packet as an array of bytes (including the header)
   * \param payload_length The length of the payload array in bytes
   */
  SickLDMessage::SickLDMessage( const uint8_t * const payload_buffer, const unsigned int payload_length ) :
    SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN >()  {

    /* Build the message object (implicit initialization) */
    BuildMessage(payload_buffer,payload_length); 
  }
  
  /**
   * \brief Another constructor.
   * \param *message_buffer A well-formed message to be parsed into the class' fields
   */
  SickLDMessage::SickLDMessage( const uint8_t * const message_buffer ) :
    SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN >()  {

    /* Parse the message into the container (implicit initialization) */
    ParseMessage(message_buffer); 
  }
  
  /**
   * \brief Constructs a Sick LD message given parameter values
   * \param *payload_buffer An address of the first byte to be copied into the message's payload
   * \param payload_length The number of bytes to be copied into the message buffer
   */
  void SickLDMessage::BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length ) {

    /* Call the parent method
     * NOTE: The parent method resets the object and assigns _message_length, _payload_length,
     *       _populated and copies the payload into the message buffer at the correct position
     */
    SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN >
      ::BuildMessage(payload_buffer,payload_length);
    
    /*
     * Set the message header!
     */
    _message_buffer[0] = 0x02; // STX
    _message_buffer[1] = 'U';  // User
    _message_buffer[2] = 'S';  // Service
    _message_buffer[3] = 'P';  // Protocol
    
    /* Include the payload length in the header */
    uint32_t payload_length_32 = host_to_sick_ld_byte_order((uint32_t)_payload_length);
    memcpy(&_message_buffer[4],&payload_length_32,4);
    
    /*
     * Set the message trailer (just a checksum)!
     */
    _message_buffer[_message_length-1] = _computeXOR(&_message_buffer[8],(uint32_t)_payload_length);
  }
  
  /**
   * \brief Parses a sequence of bytes into a SickLDMessage object
   * \param *message_buffer A well-formed message to be parsed into the class' fields
   */
  void SickLDMessage::ParseMessage( const uint8_t * const message_buffer ) {
    
    /* Call the parent method
     * NOTE: This method resets the object and assigns _populated as true
     */
    SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN >
      ::ParseMessage(message_buffer);
    
    /* Extract the payload length */
    uint32_t payload_length_32 = 0;
    memcpy(&payload_length_32,&message_buffer[4],4);
    _payload_length = (unsigned int)sick_ld_to_host_byte_order(payload_length_32);

    /* Compute the total message length */
    _message_length = MESSAGE_HEADER_LENGTH + MESSAGE_TRAILER_LENGTH + _payload_length;
    
    /* Copy the given packet into the buffer */
    memcpy(_message_buffer,message_buffer,_message_length);
  }

  /**
   * \brief Print the message contents.
   */
  void SickLDMessage::Print( ) const {

    std::cout.setf(std::ios::hex,std::ios::basefield);
    std::cout << "Checksum: " << (unsigned int) GetChecksum() << std::endl;  
    std::cout << "Service code: " << (unsigned int) GetServiceCode() << std::endl;
    std::cout << "Service subcode: " << (unsigned int) GetServiceSubcode() << std::endl;
    std::cout << std::flush;

    /* Call parent's print function */
    SickMessage< SICK_LD_MSG_HEADER_LEN, SICK_LD_MSG_PAYLOAD_MAX_LEN, SICK_LD_MSG_TRAILER_LEN >::Print();    
  }
  
  /**
   * \brief Compute the message checksum (single-byte XOR).
   * \param data The address of the first data element in a sequence of bytes to be included in the sum
   * \param length The number of byte in the data sequence
   */
  uint8_t SickLDMessage::_computeXOR( const uint8_t * const data, const uint32_t length ) {
    
    /* Compute the XOR by summing all of the bytes */
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < length; i++) {
      checksum ^= data[i]; // NOTE: this is equivalent to simply summing all of the bytes
    }
    
    /* done */
    return checksum;
  }
  
  SickLDMessage::~SickLDMessage( ) { }
  
} /* namespace SickToolbox */
